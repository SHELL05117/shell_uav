#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include "ego_path_tracking.h"

namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
}

class EgoTrajectoryController
{
public:
  EgoTrajectoryController()
      : private_nh_("~")
  {
    private_nh_.param("trajectory_topic", trajectory_topic_,
                      std::string("/visual_local_trajectory"));
    private_nh_.param("global_frame", global_frame_, std::string("map"));
    private_nh_.param("base_frame", base_frame_, std::string("base_link"));
    private_nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    private_nh_.param("trigger_topic", trigger_topic_, std::string("/trigger_plan"));
    private_nh_.param("max_velocity", max_velocity_, 0.35);
    private_nh_.param("max_acceleration", max_acceleration_, 0.50);
    private_nh_.param("lookahead_distance", lookahead_distance_, 0.40);
    private_nh_.param("goal_slowdown_distance", goal_slowdown_distance_, 0.60);
    private_nh_.param("goal_tolerance", goal_tolerance_, 0.10);
    private_nh_.param("command_timeout", command_timeout_, 0.60);
    private_nh_.param("control_rate", control_rate_, 20.0);
    max_velocity_ = std::max(0.0, max_velocity_);
    max_acceleration_ = std::max(0.0, max_acceleration_);
    lookahead_distance_ = std::max(0.01, lookahead_distance_);
    goal_tolerance_ = std::max(0.01, goal_tolerance_);
    goal_slowdown_distance_ = std::max(goal_tolerance_, goal_slowdown_distance_);
    command_timeout_ = std::max(0.05, command_timeout_);
    control_rate_ = std::max(1.0, control_rate_);
    // The published command is the PLAN-FRAME velocity, not a body-frame one.
    //
    // There used to be an output_frame switch here (body / odom), paired with
    // rescue_mission's cmd_vel_frame. The "body" option converted plan->body
    // here using yaw(odom -> base_link) and then body->plan in the mission using
    // PX4's EKF yaw - two mutually inverse rotations from two different
    // estimators, whose residual is a rotation of the velocity vector, i.e. a
    // sideways push. Publishing in odom makes the conversion happen once, here,
    // so the residual is zero by construction rather than by tuning. As of
    // 2026-10-17 the switch is gone with the DWA/TEB backends that needed it.

    trajectory_sub_ = nh_.subscribe(trajectory_topic_, 1,
                                    &EgoTrajectoryController::trajectoryCallback, this);
    trigger_sub_ = nh_.subscribe(trigger_topic_, 1,
                                 &EgoTrajectoryController::triggerCallback, this);
    command_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                     &EgoTrajectoryController::controlCallback, this);
    ROS_INFO("EGO trajectory controller ready: trajectory=%s TF=%s->%s "
             "command=%s (odom frame, no rotation) trigger=%s",
             trajectory_topic_.c_str(), global_frame_.c_str(),
             base_frame_.c_str(), cmd_vel_topic_.c_str(),
             trigger_topic_.c_str());
  }

private:
  void trajectoryCallback(const nav_msgs::Path::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    trajectory_ = *msg;
    trajectory_received_at_ = ros::Time::now();
  }

  void triggerCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Default is true, so a setup that never publishes /trigger_plan behaves
    // exactly as before. The mission node latches this topic, which is what lets
    // this node notice that the flight is over.
    planning_enabled_ = msg->data;
    ROS_INFO("EGO follower: planning %s", planning_enabled_ ? "enabled" : "disabled");
  }

  void controlCallback(const ros::TimerEvent &)
  {
    nav_msgs::Path trajectory;
    ros::Time received_at;
    bool planning_enabled = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      trajectory = trajectory_;
      received_at = trajectory_received_at_;
      planning_enabled = planning_enabled_;
    }

    geometry_msgs::Twist command;
    const ros::Time now = ros::Time::now();
    if (!planning_enabled)
    {
      // Checked before the empty-trajectory test below, because when planning is
      // disabled the planner publishes an empty path on purpose; reporting that
      // as a warning would be crying wolf. Reporting it as INFO once is enough -
      // the mission has already logged the state change.
      ROS_INFO_THROTTLE(5.0, "EGO follower: planning disabled, commanding zero velocity");
      previous_command_ = geometry_msgs::Twist();
      command_pub_.publish(command);
      return;
    }
    if (trajectory.poses.empty())
    {
      ROS_WARN_THROTTLE(2.0, "EGO planner published an empty local trajectory");
      previous_command_ = geometry_msgs::Twist();
      command_pub_.publish(command);
      return;
    }
    if (received_at.isZero() || (now - received_at).toSec() > command_timeout_)
    {
      ROS_WARN_THROTTLE(2.0, "EGO local trajectory is stale");
      previous_command_ = geometry_msgs::Twist();
      command_pub_.publish(command);
      return;
    }
    if (trajectory.header.frame_id != global_frame_)
    {
      ROS_WARN_THROTTLE(2.0, "EGO trajectory frame '%s' does not match configured frame '%s'",
                        trajectory.header.frame_id.c_str(), global_frame_.c_str());
      previous_command_ = geometry_msgs::Twist();
      command_pub_.publish(command);
      return;
    }

    tf::StampedTransform transform;
    try
    {
      tf_listener_.lookupTransform(global_frame_, base_frame_, ros::Time(0), transform);
    }
    catch (const tf::TransformException &error)
    {
      ROS_WARN_THROTTLE(1.0, "EGO follower cannot get pose TF: %s", error.what());
      previous_command_ = geometry_msgs::Twist();
      command_pub_.publish(command);
      return;
    }

    const double x = transform.getOrigin().x();
    const double y = transform.getOrigin().y();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    geometry_msgs::Point target;
    double remaining_distance = 0.0;
    double cross_track = 0.0;
    if (!ego_path_tracking::selectLookaheadPoint(
            trajectory, x, y, lookahead_distance_, goal_tolerance_,
            target, remaining_distance, &cross_track))
    {
      // This used to return completely silently, which is why "the vehicle flew
      // for a while and then just stopped" had no explanation anywhere in the
      // log. selectLookaheadPoint returns false for exactly two reasons once the
      // path is known to be non-empty: a non-finite input, or the vehicle being
      // within goal_tolerance of path.poses.back().
      //
      // The distinction matters because poses.back() is the end of the LOCAL
      // trajectory, not the route goal - EgoPlanner2DNode truncates the
      // reference path to local_window_points * path_sample_interval metres
      // ahead of the vehicle. So "arrived" here can mean "ran out of plan"
      // rather than "reached the waypoint", and only the planner's own
      // "EGO planned N point(s), end ... is X m from the goal" line can tell
      // those two apart.
      const auto &last = trajectory.poses.back().pose.position;
      const double to_end = std::hypot(last.x - x, last.y - y);
      ROS_WARN_THROTTLE(
          2.0,
          "EGO follower idling: %zu pose(s), last (%.2f, %.2f) is %.3f m away, "
          "goal_tolerance=%.2f -> %s",
          trajectory.poses.size(), last.x, last.y, to_end, goal_tolerance_,
          to_end <= goal_tolerance_
              ? "reached the plan end (this is NOT necessarily the route goal)"
              : "non-finite pose or trajectory point");
      previous_command_ = geometry_msgs::Twist();
      command_pub_.publish(command);
      return;
    }

    const double dx = target.x - x;
    const double dy = target.y - y;
    const double distance = std::hypot(dx, dy);
    const double speed_scale = std::min(1.0, remaining_distance / goal_slowdown_distance_);
    const double speed = std::min(max_velocity_, max_velocity_ * speed_scale);
    const double map_vx = speed * dx / std::max(distance, 1e-6);
    const double map_vy = speed * dy / std::max(distance, 1e-6);
    if (distance > 1e-6)
    {
      // Already in the plan frame: handed over untouched, so exactly one
      // conversion exists in the whole chain (there is none any more).
      command.linear.x = map_vx;
      command.linear.y = map_vy;
    }
    command.linear.z = 0.0;
    command.angular.z = 0.0;
    const double dt = 1.0 / control_rate_;
    const double dvx = command.linear.x - previous_command_.linear.x;
    const double dvy = command.linear.y - previous_command_.linear.y;
    const double delta_speed = std::hypot(dvx, dvy);
    const double max_delta = max_acceleration_ * dt;
    if (delta_speed > max_delta && delta_speed > 0.0)
    {
      const double scale = max_delta / delta_speed;
      command.linear.x = previous_command_.linear.x + dvx * scale;
      command.linear.y = previous_command_.linear.y + dvy * scale;
    }
    previous_command_ = command;
    // The one line that makes a sway measurable rather than guessed at. `target`
    // is the pursuit point, `delta` is the vector from the vehicle to it, `lag`
    // is how far off the plan the vehicle currently is (the distance from the
    // path, not to the target - a short lookahead turns any lag into a large
    // heading error: at lookahead 0.20 m a 0.05 m lag is already 14 deg of steer,
    // which is why the lookahead must not be shorter than roughly one second of
    // travel), and `cmd` is what leaves for the mission node.
    ROS_INFO_THROTTLE(1.0,
                      "EGO follower: pose=(%.2f, %.2f) yaw=%.1f deg lookahead=%.2f "
                      "target=(%.2f, %.2f) delta=(%.3f, %.3f) lag=%.3f rem=%.2f "
                      "speed=%.3f cmd=(%.3f, %.3f) [odom]",
                      x, y, yaw * kRadToDeg, lookahead_distance_,
                      target.x, target.y, dx, dy, cross_track, remaining_distance,
                      speed, command.linear.x, command.linear.y);
    command_pub_.publish(command);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber trigger_sub_;
  ros::Publisher command_pub_;
  ros::Timer control_timer_;
  tf::TransformListener tf_listener_;
  std::mutex mutex_;
  nav_msgs::Path trajectory_;
  geometry_msgs::Twist previous_command_;
  ros::Time trajectory_received_at_;
  // True until /trigger_plan says otherwise, so a deployment that does not use
  // that topic (RViz testing) is unaffected.
  bool planning_enabled_ = true;
  std::string trajectory_topic_;
  std::string global_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  std::string trigger_topic_ = "/trigger_plan";
  double max_velocity_ = 0.35;
  double max_acceleration_ = 0.50;
  double lookahead_distance_ = 0.40;
  double goal_slowdown_distance_ = 0.60;
  double goal_tolerance_ = 0.10;
  double command_timeout_ = 0.60;
  double control_rate_ = 20.0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_trajectory_controller");
  EgoTrajectoryController controller;
  ros::spin();
  return 0;
}
