#include <algorithm>
#include <cmath>
#include <string>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

class SimulationVehicle
{
public:
  SimulationVehicle()
      : private_nh_("~")
  {
    private_nh_.param<std::string>("model_name", model_name_, "sim_quadrotor");
    private_nh_.param<std::string>("model_state_service", model_state_service_,
                                    "/gazebo/set_model_state");
    private_nh_.param<double>("start_x", position_.x, 0.0);
    private_nh_.param<double>("start_y", position_.y, 0.0);
    private_nh_.param<double>("start_z", position_.z, 0.0);
    private_nh_.param<double>("simulation_rate", simulation_rate_, 50.0);
    private_nh_.param<double>("max_horizontal_speed", max_horizontal_speed_, 0.8);
    private_nh_.param<double>("max_vertical_speed", max_vertical_speed_, 0.5);
    private_nh_.param<double>("position_gain", position_gain_, 1.4);
    private_nh_.param<double>("goal_speed", goal_speed_, 0.55);
    private_nh_.param<double>("goal_tolerance", goal_tolerance_, 0.10);
    private_nh_.param<bool>("direct_goal_control", direct_goal_control_, false);

    state_.connected = true;
    state_.mode = "MANUAL";
    state_.armed = false;

    state_pub_ = nh_.advertise<mavros_msgs::State>("/mavros/state", 1);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/mavros/local_position/odom", 1);
    if (direct_goal_control_)
    {
      planner_velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }
    gpio_ack_pub_ = nh_.advertise<std_msgs::String>("/gpio_ack", 1);

    setpoint_sub_ = nh_.subscribe("/mavros/setpoint_raw/local", 10,
                                  &SimulationVehicle::setpointCallback, this);
    // The mission's own goal topic, not /move_base_simple/goal: mavros_extras'
    // guided_target plugin advertises that shared default and republishes the
    // FCU's POSITION_TARGET_GLOBAL_INT there as a 'map'-frame PoseStamped at
    // ~10 Hz, and the planner's frame check answers foreign goals with
    // clearPlan(). See check_config.py item 18 - both ends must stay equal.
    goal_sub_ = nh_.subscribe("/mission/route_goal", 10,
                              &SimulationVehicle::goalCallback, this);
    planner_velocity_sub_ = nh_.subscribe("/cmd_vel", 10,
                                           &SimulationVehicle::plannerVelocityCallback, this);
    gpio_sub_ = nh_.subscribe("/gpio_cmd", 10,
                              &SimulationVehicle::gpioCallback, this);

    arm_service_ = nh_.advertiseService("/mavros/cmd/arming",
                                        &SimulationVehicle::armCallback, this);
    mode_service_ = nh_.advertiseService("/mavros/set_mode",
                                         &SimulationVehicle::modeCallback, this);
    gazebo_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(
        model_state_service_);

    update_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, simulation_rate_)),
        &SimulationVehicle::update, this);

    ROS_INFO("Gazebo simulation vehicle ready: model=%s", model_name_.c_str());
  }

private:
  void setpointCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
  {
    last_setpoint_ = *msg;
    have_setpoint_ = true;
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    navigation_goal_ = msg->pose.position;
    have_navigation_goal_ = true;
  }

  void plannerVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    last_planner_velocity_ = *msg;
  }

  void gpioCallback(const std_msgs::String::ConstPtr &msg)
  {
    std_msgs::String ack;
    ack.data = msg->data + "_DONE";
    gpio_ack_pub_.publish(ack);
  }

  bool armCallback(mavros_msgs::CommandBool::Request &request,
                   mavros_msgs::CommandBool::Response &response)
  {
    if (request.value && state_.mode != "OFFBOARD")
    {
      response.success = false;
      return true;
    }
    state_.armed = request.value;
    response.success = true;
    return true;
  }

  bool modeCallback(mavros_msgs::SetMode::Request &request,
                    mavros_msgs::SetMode::Response &response)
  {
    state_.mode = request.custom_mode;
    response.mode_sent = true;
    return true;
  }

  double clamp(double value, double low, double high) const
  {
    return std::max(low, std::min(value, high));
  }

  void update(const ros::TimerEvent &event)
  {
    const double measured_dt = (event.current_real - event.last_real).toSec();
    const double dt = measured_dt > 0.0 ? std::min(0.1, measured_dt) : 0.02;
    geometry_msgs::Twist planner_velocity;

    if (direct_goal_control_ && have_navigation_goal_)
    {
      const double dx = navigation_goal_.x - position_.x;
      const double dy = navigation_goal_.y - position_.y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      if (distance > goal_tolerance_)
      {
        const double speed = std::min(goal_speed_, 0.8 * distance);
        planner_velocity.linear.x = speed * dx / distance;
        planner_velocity.linear.y = speed * dy / distance;
      }
    }
    if (direct_goal_control_)
    {
      planner_velocity_pub_.publish(planner_velocity);
    }

    if (state_.armed && have_setpoint_ &&
        (state_.mode == "OFFBOARD" || state_.mode == "AUTO.LAND"))
    {
      double vx = 0.0;
      double vy = 0.0;
      double vz = 0.0;

      if (!(last_setpoint_.type_mask & mavros_msgs::PositionTarget::IGNORE_VX))
      {
        vx = last_setpoint_.velocity.x;
      }
      if (!(last_setpoint_.type_mask & mavros_msgs::PositionTarget::IGNORE_VY))
      {
        vy = last_setpoint_.velocity.y;
      }
      if (!(last_setpoint_.type_mask & mavros_msgs::PositionTarget::IGNORE_VZ))
      {
        vz = last_setpoint_.velocity.z;
      }

      if (!(last_setpoint_.type_mask & mavros_msgs::PositionTarget::IGNORE_PX))
      {
        vx += position_gain_ * (last_setpoint_.position.x - position_.x);
      }
      if (!(last_setpoint_.type_mask & mavros_msgs::PositionTarget::IGNORE_PY))
      {
        vy += position_gain_ * (last_setpoint_.position.y - position_.y);
      }
      if (!(last_setpoint_.type_mask & mavros_msgs::PositionTarget::IGNORE_PZ))
      {
        vz += position_gain_ * (last_setpoint_.position.z - position_.z);
      }

      if (state_.mode == "AUTO.LAND")
      {
        vz = -max_vertical_speed_;
      }

      const double horizontal_speed = std::sqrt(vx * vx + vy * vy);
      if (horizontal_speed > max_horizontal_speed_)
      {
        const double scale = max_horizontal_speed_ / horizontal_speed;
        vx *= scale;
        vy *= scale;
      }
      vz = clamp(vz, -max_vertical_speed_, max_vertical_speed_);

      position_.x += vx * dt;
      position_.y += vy * dt;
      position_.z = std::max(0.0, position_.z + vz * dt);

      if (state_.mode == "AUTO.LAND" && position_.z <= 0.03)
      {
        position_.z = 0.0;
        state_.armed = false;
      }
    }

    publishState();
    publishPose();
    if (++gazebo_update_tick_ >= 5)
    {
      gazebo_update_tick_ = 0;
      updateGazeboModel();
    }
  }

  void publishState()
  {
    state_.header.stamp = ros::Time::now();
    state_pub_.publish(state_);
  }

  void publishPose()
  {
    const ros::Time stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose.position = position_;
    pose.pose.orientation.w = 1.0;
    pose_pub_.publish(pose);

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose = pose.pose;
    odom.pose.pose.position = position_;
    odom_pub_.publish(odom);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(position_.x, position_.y, position_.z));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(transform, stamp, "map", "base_link"));
  }

  void updateGazeboModel()
  {
    if (!gazebo_state_client_.exists())
    {
      return;
    }

    gazebo_msgs::SetModelState request;
    request.request.model_state.model_name = model_name_;
    request.request.model_state.pose.position = position_;
    request.request.model_state.pose.orientation.w = 1.0;
    request.request.model_state.reference_frame = "world";
    if (!gazebo_state_client_.call(request) && !warned_gazebo_service_)
    {
      ROS_WARN("Gazebo model state service is unavailable for '%s'",
               model_name_.c_str());
      warned_gazebo_service_ = true;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher state_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher planner_velocity_pub_;
  ros::Publisher gpio_ack_pub_;
  ros::Subscriber setpoint_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber planner_velocity_sub_;
  ros::Subscriber gpio_sub_;
  ros::ServiceServer arm_service_;
  ros::ServiceServer mode_service_;
  ros::ServiceClient gazebo_state_client_;
  ros::Timer update_timer_;
  tf::TransformBroadcaster tf_broadcaster_;

  mavros_msgs::State state_;
  mavros_msgs::PositionTarget last_setpoint_;
  geometry_msgs::Point position_;
  geometry_msgs::Point navigation_goal_;
  geometry_msgs::Twist last_planner_velocity_;
  std::string model_name_;
  std::string model_state_service_;
  bool have_setpoint_ = false;
  bool have_navigation_goal_ = false;
  bool direct_goal_control_ = false;
  bool warned_gazebo_service_ = false;
  int gazebo_update_tick_ = 0;
  double simulation_rate_ = 50.0;
  double max_horizontal_speed_ = 0.8;
  double max_vertical_speed_ = 0.5;
  double position_gain_ = 1.4;
  double goal_speed_ = 0.55;
  double goal_tolerance_ = 0.10;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_simulation_vehicle");
  SimulationVehicle vehicle;
  ros::spin();
  return 0;
}
