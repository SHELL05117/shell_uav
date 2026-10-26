/***************************************************************************************************************************
 *
 * Author: Amy
 * Time: 2023.02.02
 * Description: set arm + set mode + flight control for px4 flight stack
 ***************************************************************************************************************************/

#include "flight_control.h"
#include <cmath>

namespace
{
// publish() 前的最后一道保险。一个没建起来的发布者在 ros::Publisher::publish()
// 里是 assert，直接 abort 整个节点；而少发一帧设定值最多让飞机多悬停 20 ms。
// 真机上宁愿丢一帧，也不能因为一个配置错误把飞机和任务一起带走。
bool setpoint_publisher_ready(const ros::Publisher &publisher, const char *topic)
{
    if (publisher)
        return true;
    ROS_ERROR_ONCE("FlightControl: %s has no publisher; setpoint dropped", topic);
    return false;
}

void publish_raw(ros::Publisher &publisher, mavros_msgs::PositionTarget &message)
{
    if (!setpoint_publisher_ready(publisher, "/mavros/setpoint_raw/local"))
        return;
    const uint16_t mask = message.type_mask;
    const bool valid =
        ((mask & PositionTarget::IGNORE_PX) || std::isfinite(message.position.x)) &&
        ((mask & PositionTarget::IGNORE_PY) || std::isfinite(message.position.y)) &&
        ((mask & PositionTarget::IGNORE_PZ) || std::isfinite(message.position.z)) &&
        ((mask & PositionTarget::IGNORE_VX) || std::isfinite(message.velocity.x)) &&
        ((mask & PositionTarget::IGNORE_VY) || std::isfinite(message.velocity.y)) &&
        ((mask & PositionTarget::IGNORE_VZ) || std::isfinite(message.velocity.z)) &&
        ((mask & PositionTarget::IGNORE_AFX) || std::isfinite(message.acceleration_or_force.x)) &&
        ((mask & PositionTarget::IGNORE_AFY) || std::isfinite(message.acceleration_or_force.y)) &&
        ((mask & PositionTarget::IGNORE_AFZ) || std::isfinite(message.acceleration_or_force.z)) &&
        ((mask & PositionTarget::IGNORE_YAW) || std::isfinite(message.yaw)) &&
        ((mask & PositionTarget::IGNORE_YAW_RATE) || std::isfinite(message.yaw_rate));
    if (!valid)
    {
        ROS_ERROR_THROTTLE(1.0, "FlightControl: non-finite active setpoint rejected");
        return;
    }
    message.header.stamp = ros::Time::now();
    publisher.publish(message);
}
}  // namespace

// 默认构造必须和带命名空间的那个一样把发布者建起来。
//
// MoveBaseMission 的成员写的是 `FlightControl flightControl;`，走的就是这个默认
// 构造函数。它以前是空实现（`{}`），于是 /mavros/setpoint_raw/local 从来没有被
// advertise 过；50 Hz 控制环在收到第一帧 /mavros/local_position/pose 之后的第一次
// publish 就撞上
//     [FATAL] ASSERTION FAILED
//             file = /opt/ros/noetic/include/ros/publisher.h, line = 110
//             message = Call to publish() on an invalid Publisher
// 任务节点直接 abort（exit code -5），而 navigation_2d.launch 里它是
// required="true"，于是 roslaunch 把 Gazebo、PX4、cartographer 一起带走 ——
// 现象就是"仿真还没起飞就整体退出"。委托给带命名空间的构造函数后，发布者一定存在。
FlightControl::FlightControl() : FlightControl("") {}

FlightControl::FlightControl(string mavros_ns) : mavros_ns_(mavros_ns)
{
    nh_ = ros::NodeHandle("");
    setpoint_position_local_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
}

FlightControl::~FlightControl()
{
    // Destructor
}

void FlightControl::set_arm(bool is_arm)
{
    CommandBool arm_msg;
    arm_msg.request.value = is_arm;

    ros::ServiceClient set_arm_client = nh_.serviceClient<CommandBool>(mavros_ns_ + "/mavros/cmd/arming");
    if (!set_arm_client.call(arm_msg))
    {
        ROS_ERROR("Failed to call service arming");
    }
    else if (!arm_msg.response.success)
    {
        ROS_WARN("PX4 rejected arming=%s, MAV_RESULT=%u; inspect /mavros/statustext/recv",
                 is_arm ? "true" : "false", static_cast<unsigned>(arm_msg.response.result));
    }
    else
    {
        ROS_INFO("PX4 accepted arming=%s", is_arm ? "true" : "false");
    }
}

void FlightControl::set_mode(string mode)
{
    mavros_msgs::SetMode mode_msg;
    mode_msg.request.custom_mode = mode;

    ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>(mavros_ns_ + "/mavros/set_mode");
    if (!set_mode_client.call(mode_msg))
    {
        ROS_ERROR("Failed to call service set_mode: %s", mode.c_str());
    }
    else if (!mode_msg.response.mode_sent)
    {
        ROS_WARN("MAVROS did not send mode request: %s", mode.c_str());
    }
}

// void PX4RosNav::set_mode(string mode)
// {
//     mavros_msgs::SetMode set_mode_msg;
//     set_mode_msg.request.custom_mode = mode;

//     ros::ServiceClient set_mode_client = nh_private_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
//     while (!set_mode_client.call(set_mode_msg))
//     {
//     }
//     if (set_mode_msg.response.mode_sent)
//     {
//         ROS_WARN("Set Mode: %s Finished", modes.c_str());
//     }
// }

void FlightControl::setpoint_position_local(const geometry_msgs::Point pos)
{
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z))
    {
        ROS_ERROR_THROTTLE(1.0, "FlightControl: non-finite position rejected");
        return;
    }
    geometry_msgs::PoseStamped poseTarget;
    poseTarget.header.stamp = ros::Time::now();
    poseTarget.pose.position = pos;
    poseTarget.pose.orientation.w = 1.0;
    if (!setpoint_publisher_ready(setpoint_position_local_pub_, "/mavros/setpoint_position/local"))
        return;
    setpoint_position_local_pub_.publish(poseTarget);
}

void FlightControl::setpoint_raw_local_pos(const geometry_msgs::Point pos, float yaw, int frame)
{
    // ROS_WARN_ONCE("[setpoint_raw_local_pos] yaw_rate = %f,frame = %d.", yaw_rate, frame);
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.header.stamp = ros::Time::now();
    positionTarget.type_mask =
        PositionTarget::IGNORE_VX + PositionTarget::IGNORE_VY + PositionTarget::IGNORE_VZ +
        PositionTarget::IGNORE_AFX + PositionTarget::IGNORE_AFY + PositionTarget::IGNORE_AFZ +
        PositionTarget::FORCE +
        PositionTarget::IGNORE_YAW_RATE;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED or PositionTarget::FRAME_BODY_NED

    positionTarget.position = pos;
    positionTarget.yaw = yaw;

    publish_raw(setpoint_raw_local_pub_, positionTarget);
}

void FlightControl::setpoint_raw_local_vel(const geometry_msgs::Point vel, float yaw_rate, int frame)
{
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.type_mask =
        PositionTarget::IGNORE_PX + PositionTarget::IGNORE_PY + PositionTarget::IGNORE_PZ +
        PositionTarget::IGNORE_AFX + PositionTarget::IGNORE_AFY + PositionTarget::IGNORE_AFZ +
        PositionTarget::FORCE +
        PositionTarget::IGNORE_YAW;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED or PositionTarget::FRAME_BODY_NED

    positionTarget.velocity.x = vel.x;
    positionTarget.velocity.y = vel.y;
    positionTarget.velocity.z = vel.z;
    positionTarget.yaw_rate = yaw_rate;
    publish_raw(setpoint_raw_local_pub_, positionTarget);
}

void FlightControl::setpoint_raw_local_pos_vel(const geometry_msgs::Point pos, const geometry_msgs::Point vel, float yaw, int frame)
{
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.type_mask =
        PositionTarget::IGNORE_AFX + PositionTarget::IGNORE_AFY + PositionTarget::IGNORE_AFZ +
        PositionTarget::FORCE +
        PositionTarget::IGNORE_YAW_RATE;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED(default) or PositionTarget::FRAME_BODY_NED

    positionTarget.position = pos;

    positionTarget.velocity.x = vel.x;
    positionTarget.velocity.y = vel.y;
    positionTarget.velocity.z = vel.z;

    positionTarget.yaw = yaw;
    publish_raw(setpoint_raw_local_pub_, positionTarget);
}

void FlightControl::setpoint_raw_local_pos_vel_acc(const geometry_msgs::Point pos, const geometry_msgs::Point vel, const geometry_msgs::Point acc, float yaw, int frame)
{
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.type_mask =
        PositionTarget::FORCE +
        PositionTarget::IGNORE_YAW_RATE;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED(default) or PositionTarget::FRAME_BODY_NED

    positionTarget.position = pos;

    positionTarget.velocity.x = vel.x;
    positionTarget.velocity.y = vel.y;
    positionTarget.velocity.z = vel.z;

    positionTarget.acceleration_or_force.x = acc.x;
    positionTarget.acceleration_or_force.y = acc.y;
    positionTarget.acceleration_or_force.z = acc.z;

    positionTarget.yaw = yaw;
    publish_raw(setpoint_raw_local_pub_, positionTarget);
}

void FlightControl::setpoint_raw_local_pos_vel_yawrate(const geometry_msgs::Point pos, const geometry_msgs::Point vel, float yaw, float yaw_rate, int frame)
{
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.type_mask =
        PositionTarget::IGNORE_AFX + PositionTarget::IGNORE_AFY + PositionTarget::IGNORE_AFZ +
        PositionTarget::FORCE;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED(default) or PositionTarget::FRAME_BODY_NED

    positionTarget.position = pos;

    positionTarget.velocity.x = vel.x;
    positionTarget.velocity.y = vel.y;
    positionTarget.velocity.z = vel.z;

    positionTarget.yaw = yaw;
    positionTarget.yaw_rate = yaw_rate;
    publish_raw(setpoint_raw_local_pub_, positionTarget);
}

void FlightControl::setpoint_raw_local_pos_vel_acc_yawrate(const geometry_msgs::Point pos, const geometry_msgs::Vector3 vel, const geometry_msgs::Vector3 acc, float yaw, float yaw_rate, int frame)
{
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.type_mask =
        PositionTarget::FORCE;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED(default) or PositionTarget::FRAME_BODY_NED

    positionTarget.position = pos;

    positionTarget.velocity.x = vel.x;
    positionTarget.velocity.y = vel.y;
    positionTarget.velocity.z = vel.z;

    positionTarget.acceleration_or_force.x = acc.x;
    positionTarget.acceleration_or_force.y = acc.y;
    positionTarget.acceleration_or_force.z = acc.z;

    positionTarget.yaw = yaw;
    positionTarget.yaw_rate = yaw_rate;
    publish_raw(setpoint_raw_local_pub_, positionTarget);
}

void FlightControl::setpoint_raw_local_velxy_posz(const geometry_msgs::Point vel_xy_pos_z, float yaw, int frame)
{
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.type_mask =
        PositionTarget::IGNORE_PX + PositionTarget::IGNORE_PY +
        PositionTarget::IGNORE_AFX + PositionTarget::IGNORE_AFY + PositionTarget::IGNORE_AFZ +
        PositionTarget::FORCE +
        PositionTarget::IGNORE_YAW_RATE;

    positionTarget.coordinate_frame = frame; // PositionTarget::FRAME_LOCAL_NED or PositionTarget::FRAME_BODY_NED
    positionTarget.position.z = vel_xy_pos_z.z;

    positionTarget.velocity.x = vel_xy_pos_z.x;
    positionTarget.velocity.y = vel_xy_pos_z.y;

    positionTarget.yaw = yaw;
    publish_raw(setpoint_raw_local_pub_, positionTarget);
}
