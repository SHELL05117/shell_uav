/***************************************************************************************************************************
 *
 * Author: expload
 * Time: 2026.10
 * Description: 见 mission_with_move_base.h。
 ***************************************************************************************************************************/
#include "mission_with_move_base.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <sstream>

namespace
{
const cv::Scalar kRed(0, 0, 255);    // BGR
const cv::Scalar kBlue(255, 0, 0);   // BGR

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}
}  // namespace

MoveBaseMission::MoveBaseMission(const ros::NodeHandle &nh_private, int argc, char **argv)
    : nh_private_(nh_private),
      move_base_("move_base", true),
      detect_(argc, argv)
{
  loadParams(argc, argv);

  buildRoute("hazard", "hazard_route_num", "hazard_detect_num", hazard_route_);
  buildRoute("person", "person_route_num", "person_detect_num", person_route_);
  buildRoute("return", "return_route_num", "return_detect_num", return_route_);
  hazard_detect_num_ = std::min(hazard_detect_num_, static_cast<int>(hazard_route_.size()));
  person_detect_num_ = std::min(person_detect_num_, static_cast<int>(person_route_.size()));

  logRoute(hazard_route_, "hazard", field_yaw_deg_);
  logRoute(person_route_, "person", field_yaw_deg_);
  logRoute(return_route_, "return", field_yaw_deg_);

  state_sub_ = nh_private_.subscribe("/mavros/state", 1, &MoveBaseMission::StateCallback, this,
                                     ros::TransportHints().tcpNoDelay());
  pose_sub_ = nh_private_.subscribe("/mavros/local_position/pose", 1, &MoveBaseMission::PoseCallback,
                                    this, ros::TransportHints().tcpNoDelay());
  cmd_vel_sub_ = nh_private_.subscribe("/cmd_vel", 1, &MoveBaseMission::PlannerVelCallback, this,
                                       ros::TransportHints().tcpNoDelay());
  ack_sub_ = nh_private_.subscribe("/gpio_ack", 4, &MoveBaseMission::AckCallback, this);
  target_sub_ = nh_private_.subscribe(target_topic_, 1, &MoveBaseMission::TargetCallback, this);

  status_pub_ = nh_private_.advertise<std_msgs::String>("/mission/status", 4, true);
  gpio_cmd_pub_ = nh_private_.advertise<std_msgs::String>("/gpio_cmd", 4);

  control_timer_ = nh_private_.createTimer(ros::Duration(1.0 / std::max(1.0, pub_rate_)),
                                           &MoveBaseMission::ControlLoop, this);
  tick_timer_ = nh_private_.createTimer(ros::Duration(0.1), &MoveBaseMission::TickLoop, this);

  move_base_ready_ = move_base_.waitForServer(ros::Duration(3.0));
  if (!move_base_ready_)
    ROS_WARN("[mission] move_base action server is not up yet; will keep waiting");

  phase_start_ = ros::Time::now();
  setStatus("WAIT_FCU");
  ROS_INFO("[mission] stage=%d rescue_drop=%s field_yaw=%.1f deg auto_start=%s", stage_,
           rescue_drop_ ? "yes" : "no", field_yaw_deg_, auto_start_ ? "yes" : "no");
  ROS_INFO("[mission] hazard target: %s",
           target_hazard_.empty() ? (wait_for_target_ ? "waiting (topic/param)" : "ANY (not set)")
                                  : target_hazard_.c_str());
}

MoveBaseMission::~MoveBaseMission()
{
  if (goal_active_)
    move_base_.cancelGoal();
}

void MoveBaseMission::loadParams(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  nh_private_.param("fly_height", fly_height_, 1.5);
  nh_private_.param("detect_height", detect_height_, 1.2);
  nh_private_.param("drop_height", drop_height_, 0.6);
  nh_private_.param("takeoff_height_tolerance", takeoff_height_tolerance_, 0.12);
  nh_private_.param("settle_time", settle_time_, 1.0);
  nh_private_.param("goal_timeout", goal_timeout_, 25.0);
  nh_private_.param("drop_settle_time", drop_settle_time_, 0.8);
  nh_private_.param("ack_timeout", ack_timeout_, 3.0);
  nh_private_.param("land_settle_time", land_settle_time_, 2.0);
  nh_private_.param("max_vel", max_vel_, 0.35);
  nh_private_.param("xy_gain_p", xy_gain_p_, 1.2);
  nh_private_.param("z_gain_p", z_gain_p_, 1.0);
  nh_private_.param("pub_rate", pub_rate_, 50.0);

  nh_private_.param("auto_start", auto_start_, true);
  nh_private_.param("stage", stage_, 1);
  nh_private_.param("rescue_drop_enabled", rescue_drop_enabled_, false);
  rescue_drop_ = (stage_ >= 2) || rescue_drop_enabled_;

  nh_private_.param("field_yaw_deg", field_yaw_deg_, -90.0);
  nh_private_.param("enforce_safe_field", enforce_safe_field_, true);
  nh_private_.param("field_min_x", field_min_x_, -0.55);
  nh_private_.param("field_max_x", field_max_x_, 3.25);
  nh_private_.param("field_min_y", field_min_y_, -3.25);
  nh_private_.param("field_max_y", field_max_y_, 0.55);

  std::string hazard_param;
  nh_private_.param("target_hazard", hazard_param, std::string(""));
  nh_private_.param("target_person", target_person_, std::string("person_injured"));
  nh_private_.param("target_topic", target_topic_, std::string("/mission/target_hazard"));
  nh_private_.param("wait_for_target", wait_for_target_, true);

  if (!hazard_param.empty())
  {
    target_hazard_ = resolveHazardTarget(hazard_param);
    if (target_hazard_.empty())
      ROS_ERROR("[mission] target_hazard '%s' is not a label in labels.txt and is not a known "
                "Chinese alias; refusing to guess. Use e.g. flammable / 易燃, or leave it "
                "empty and publish %s",
                hazard_param.c_str(), target_topic_.c_str());
  }
  if (!target_person_.empty() && !detect_.hasLabel(target_person_))
    ROS_ERROR("[mission] target_person '%s' is not in labels.txt", target_person_.c_str());
  if (target_hazard_.empty() && !wait_for_target_)
    ROS_WARN("[mission] wait_for_target=false and no target_hazard: the first hazard class "
             "detected will be treated as the correct one");  nh_private_.param("detect_times", detect_times_, 5);
  nh_private_.param("detect_timeout", detect_timeout_, 8.0);
  nh_private_.param("annotate_hold", annotate_hold_, 4.0);
  nh_private_.param("max_center_offset", max_center_offset_, 0.35);

  nh_private_.param("drop_align_mode", drop_align_mode_, std::string("vision_with_fallback"));
  nh_private_.param("max_align_offset", max_align_offset_, 0.35);

  double rescue_x = 999.0, rescue_y = 999.0;
  nh_private_.param("rescue_drop_x", rescue_x, 999.0);
  nh_private_.param("rescue_drop_y", rescue_y, 999.0);
  if (std::fabs(rescue_x) < 900.0 && std::fabs(rescue_y) < 900.0)
  {
    rescue_drop_field_.x = rescue_x;
    rescue_drop_field_.y = rescue_y;
    rescue_drop_field_set_ = true;
  }

  nh_private_.param("hazard_detect_num", hazard_detect_num_, 0);
  nh_private_.param("person_detect_num", person_detect_num_, 0);

  planner_velxy_posz_ = geometry_msgs::Point();
  target_position_ = geometry_msgs::Point();

  if (drop_align_mode_ != "vision" && drop_align_mode_ != "fixed" &&
      drop_align_mode_ != "vision_with_fallback")
  {
    ROS_WARN("[mission] unknown drop_align_mode '%s'; using vision_with_fallback",
             drop_align_mode_.c_str());
    drop_align_mode_ = "vision_with_fallback";
  }
}

void MoveBaseMission::buildRoute(const std::string &prefix, const std::string &count_key,
                                 const std::string &detect_key, std::vector<Waypoint> &route)
{
  int count = 0;
  int detect_num = 0;
  nh_private_.param(count_key, count, 0);
  nh_private_.param(detect_key, detect_num, 0);
  route.clear();
  for (int i = 0; i < count; ++i)
  {
    double x = 0.0, y = 0.0;
    nh_private_.param(prefix + "_point" + std::to_string(i) + "_x", x, 0.0);
    nh_private_.param(prefix + "_point" + std::to_string(i) + "_y", y, 0.0);
    Waypoint wp;
    FieldPoint field;
    field.x = x;
    field.y = y;
    wp.field = clampField(field);
    const FieldPoint map = fieldToMap(wp.field);
    wp.map_x = map.x;
    wp.map_y = map.y;
    wp.name = prefix + std::to_string(i);
    wp.detect = (detect_num > 0 && i >= count - detect_num);
    route.push_back(wp);
  }
  if (count > 0 && detect_num > count)
    ROS_WARN("[mission] %s: %d detect points but only %d route points", prefix.c_str(),
             detect_num, count);
}

void MoveBaseMission::StateCallback(const mavros_msgs::State &msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const bool was_armed = fcu_state_.armed;
  const std::string old_mode = fcu_state_.mode;
  fcu_state_ = msg;
  fcu_connected_ = msg.connected;

  if (!msg.connected)
    return;
  if (msg.armed != was_armed)
    ROS_INFO("[mission] FCU armed=%d", msg.armed);
  if (msg.mode != old_mode)
    ROS_INFO("[mission] FCU mode: %s", msg.mode.c_str());
}

void MoveBaseMission::PoseCallback(const geometry_msgs::PoseStamped &msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_pose_ = msg;
  if (!has_pose_)
  {
    has_pose_ = true;
    hold_yaw_ = yawOf(msg.pose.orientation);
    target_position_ = msg.pose.position;
    ROS_INFO("[mission] first pose: (%.2f, %.2f, %.2f), yaw %.1f deg -> holding this heading",
             msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
             hold_yaw_ * 180.0 / M_PI);
  }
}

void MoveBaseMission::TargetCallback(const std_msgs::String &msg)
{
  if (phase_ != Phase::WAIT_FCU)
  {
    ROS_WARN("[mission] %s: target can only be set before take-off (phase is no longer WAIT_FCU)",
             target_topic_.c_str());
    return;
  }
  const std::string resolved = resolveHazardTarget(msg.data);
  if (resolved.empty())
  {
    ROS_WARN("[mission] %s: '%s' is not a class in labels.txt and not a known alias",
             target_topic_.c_str(), msg.data.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_hazard_ = resolved;
  }
  setStatus("TARGET " + resolved);
  ROS_INFO("[mission] hazard target set from %s: %s", target_topic_.c_str(), resolved.c_str());
}

void MoveBaseMission::PlannerVelCallback(const geometry_msgs::Twist &msg)
{
  // move_base 给的是机体系速度；本机全程把 yaw 锁在起飞航向（≈0），机体系与 odom
  // 重合，所以直接当 local 速度用。这里只做限速，避免 DWA 的峰值速度绕过 ~max_vel。
  planner_velxy_posz_.x = clamp(msg.linear.x, -max_vel_, max_vel_);
  planner_velxy_posz_.y = clamp(msg.linear.y, -max_vel_, max_vel_);
}

void MoveBaseMission::AckCallback(const std_msgs::String &msg)
{
  if (msg.data == "DROP_FIRE_DONE" || msg.data == "DROP_RESCUE_DONE")
  {
    ack_received_ = true;
    ROS_INFO("[mission] servo ack: %s", msg.data.c_str());
  }
  else
  {
    ROS_INFO("[mission] servo ack: %s", msg.data.c_str());
  }
}

// ---------------------------------------------------------------------------
// control loop (50 Hz)
// ---------------------------------------------------------------------------

void MoveBaseMission::ControlLoop(const ros::TimerEvent &event)
{
  (void)event;
  if (!has_pose_)
    return;
  const geometry_msgs::PoseStamped pose = poseSnapshot();

  switch (control_mode_)
  {
    case ControlMode::HOLD:
    {
      const double dx = target_position_.x - pose.pose.position.x;
      const double dy = target_position_.y - pose.pose.position.y;
      const double dz = target_position_.z - pose.pose.position.z;

      geometry_msgs::Point vel;
      vel.x = clamp(xy_gain_p_ * dx, -max_vel_, max_vel_);
      vel.y = clamp(xy_gain_p_ * dy, -max_vel_, max_vel_);
      vel.z = clamp(z_gain_p_ * dz, -max_vel_, max_vel_);

      geometry_msgs::Point point;
      point.x = pose.pose.position.x + vel.x / pub_rate_ * 10.0;
      point.y = pose.pose.position.y + vel.y / pub_rate_ * 10.0;
      point.z = pose.pose.position.z + vel.z / pub_rate_ * 10.0;

      flightControl.setpoint_raw_local_pos_vel(point, vel, static_cast<float>(hold_yaw_));
      break;
    }
    case ControlMode::TRACK:
      planner_velxy_posz_.z = fly_height_;
      flightControl.setpoint_raw_local_velxy_posz(planner_velxy_posz_,
                                                  static_cast<float>(hold_yaw_));
      break;
    case ControlMode::IDLE:
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// mission tick (10 Hz)
// ---------------------------------------------------------------------------

void MoveBaseMission::TickLoop(const ros::TimerEvent &event)
{
  (void)event;
  const ros::Time now = ros::Time::now();
  const mavros_msgs::State state = stateSnapshot();
  const geometry_msgs::PoseStamped pose = poseSnapshot();

  if (!has_pose_)
    return;

  switch (phase_)
  {
    case Phase::WAIT_FCU:
    {
      holdAt(pose.pose.position);
      if (!move_base_ready_ && move_base_.isServerConnected())
      {
        move_base_ready_ = true;
        ROS_INFO("[mission] move_base connected");
      }
      if (!state.connected)
        return;
      if (!move_base_ready_)
      {
        // 没有规划器就不起飞：起飞后第一个航点会永远等不到结果
        ROS_WARN_THROTTLE(2.0, "[mission] move_base is not connected yet; refusing to take off");
        return;
      }
      bool target_missing = false;
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_missing = target_hazard_.empty();
      }
      if (target_missing && wait_for_target_)
      {
        // 危险品标志每轮随机分配：没拿到本轮的正确类别就不起飞，避免按旧默认值乱投
        ROS_WARN_THROTTLE(2.0,
                          "[mission] waiting for the hazard target; publish it on %s "
                          "(e.g. '易燃' / 'hazard_toxic') or set ~target_hazard",
                          target_topic_.c_str());
        return;
      }
      if (auto_start_)
      {
        if (state.mode != "OFFBOARD")
        {
          if (now - last_mode_request_ > ros::Duration(1.0))
          {
            requestMode("OFFBOARD");
            last_mode_request_ = now;
          }
        }
        else if (!state.armed)
        {
          if (now - last_arm_request_ > ros::Duration(1.0))
          {
            ROS_INFO("[mission] arming");
            flightControl.set_arm(true);
            last_arm_request_ = now;
          }
        }
        else
        {
          beginTakeoff();
        }
      }
      else if (state.mode == "OFFBOARD" && state.armed)
      {
        beginTakeoff();
      }
      break;
    }

    case Phase::TAKEOFF:
    {
      target_position_.x = 0.0;
      target_position_.y = 0.0;
      target_position_.z = fly_height_;
      if ((atHeight(fly_height_, takeoff_height_tolerance_) && now - phase_start_ > ros::Duration(2.0)) ||
          now - phase_start_ > ros::Duration(20.0))
      {
        if (now - phase_start_ > ros::Duration(20.0))
          ROS_WARN("[mission] take-off timeout at z=%.2f; continuing", pose.pose.position.z);
        sendGpio("CAMERA_DOWN");
        startTask(Task::HAZARD);
      }
      break;
    }

    case Phase::TRANSIT:
    {
      if (!move_base_ready_)
      {
        holdAt(pose.pose.position);
        return;
      }
      const std::vector<Waypoint> &route = currentRoute();
      if (leg_ >= route.size())
      {
        nextLeg();
        return;
      }
      if (!goal_active_)
      {
        sendGoal(route[leg_]);
        return;
      }
      if (move_base_.getState().isDone())
      {
        const actionlib::SimpleClientGoalState goal_state = move_base_.getState();
        goal_active_ = false;
        if (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          holdAt(pose.pose.position);
          onArrived();
        }
        else
        {
          ROS_WARN("[mission] move_base finished %s with state %s", route[leg_].name.c_str(),
                   goal_state.toString().c_str());
          if (goal_attempts_ < 2)
            sendGoal(route[leg_]);
          else
            nextLeg();
        }
        return;
      }
      if (now - goal_sent_ > ros::Duration(goal_timeout_))
      {
        ROS_WARN("[mission] goal %s timed out after %.0f s; cancelling", route[leg_].name.c_str(),
                 goal_timeout_);
        move_base_.cancelGoal();
        goal_active_ = false;
        if (goal_attempts_ < 2)
          sendGoal(route[leg_]);
        else
          nextLeg();
      }
      break;
    }

    case Phase::SETTLE:
    {
      control_mode_ = ControlMode::HOLD;
      target_position_.z = detect_height_;
      const bool settled = atHeight(detect_height_, 0.15) &&
                           now - phase_start_ > ros::Duration(settle_time_);
      if (settled || now - phase_start_ > ros::Duration(settle_time_ + 6.0))
        beginDetect();
      break;
    }

    case Phase::DETECT:
    {
      if (now - phase_start_ > ros::Duration(detect_timeout_))
      {
        onDetectFailed();
        return;
      }
      if (now - last_infer_ < ros::Duration(0.12))
        return;
      last_infer_ = now;

      Detect::Detection best;
      cv::Point2f body;
      if (refreshDetectTarget(best, body))
      {
        hits_.push_back(Hit{now, best.center, best});
        detect_.annotate(best, task_ == Task::HAZARD ? kRed : kBlue,
                         task_ == Task::HAZARD ? "hazard" : "person", annotate_hold_);
        ROS_INFO_THROTTLE(1.0, "[mission] %s hit: %s %.2f offset=(%.2f, %.2f) [%zu/%d]",
                          task_ == Task::HAZARD ? "hazard" : "person", best.label.c_str(),
                          best.confidence, body.x, body.y, hits_.size(), detect_times_);
      }
      while (!hits_.empty() && now - hits_.front().stamp > ros::Duration(1.5))
        hits_.pop_front();

      if (static_cast<int>(hits_.size()) >= detect_times_)
      {
        cv::Point2f mean(0.0f, 0.0f);
        for (const auto &hit : hits_)
          mean += hit.center;
        mean *= 1.0f / static_cast<float>(hits_.size());

        const double height = std::max(0.2, pose.pose.position.z - 0.02);
        cv::Point2f mean_body;
        if (!detect_.pixelToBody(mean, height, mean_body))
          mean_body = body;
        onConfirmed(hits_.back().det, mean_body);
      }
      break;
    }

    case Phase::DESCEND:
    {
      control_mode_ = ControlMode::HOLD;
      target_position_ = drop_target_;
      target_position_.z = drop_height_;
      const double xy_error = std::hypot(pose.pose.position.x - drop_target_.x,
                                         pose.pose.position.y - drop_target_.y);
      if ((atHeight(drop_height_, 0.12) && xy_error < 0.15) ||
          now - phase_start_ > ros::Duration(10.0))
      {
        if (now - phase_start_ > ros::Duration(10.0))
          ROS_WARN("[mission] descent timeout at z=%.2f xy_err=%.2f; settling anyway",
                   pose.pose.position.z, xy_error);
        phase_ = Phase::DROP;
        phase_start_ = now;
        drop_sent_ = false;
        ack_received_ = false;
      }
      break;
    }

    case Phase::DROP:
    {
      holdAt(drop_target_);
      if (!drop_sent_)
      {
        // 在投放高度先稳一下再开仓：下降过程中机体还在摆，早开仓会把物资甩出去
        if (now - phase_start_ >= ros::Duration(drop_settle_time_))
        {
          const std::string command = drop_is_fire_ ? "DROP_FIRE" : "DROP_RESCUE";
          sendGpio(command);
          setStatus(command);
          drop_sent_ = true;
          drop_cmd_time_ = now;
        }
        break;
      }
      if (ack_received_)
      {
        ack_received_ = false;
        setStatus("DROP_ACK");
        beginClimb();
      }
      else if (now - drop_cmd_time_ > ros::Duration(ack_timeout_))
      {
        ROS_WARN("[mission] no /gpio_ack within %.1f s; climbing anyway", ack_timeout_);
        beginClimb();
      }
      break;
    }

    case Phase::CLIMB:
    {
      control_mode_ = ControlMode::HOLD;
      target_position_.x = drop_target_.x;
      target_position_.y = drop_target_.y;
      target_position_.z = fly_height_;
      if (atHeight(fly_height_, 0.15) || now - phase_start_ > ros::Duration(8.0))
      {
        if (after_drop_task_ == Task::PERSON)
          startTask(Task::PERSON);
        else
          startTask(Task::RETURN);
      }
      break;
    }

    case Phase::LAND:
    {
      holdAt(pose.pose.position);
      if (now - phase_start_ < ros::Duration(land_settle_time_))
        return;

      if (land_stage_ == 0)
      {
        requestMode("POSCTL");
        land_stage_ = 1;
        land_stage_time_ = now;
      }
      else if (land_stage_ == 1 && now - land_stage_time_ > ros::Duration(0.5))
      {
        requestMode("AUTO.LAND");
        land_stage_ = 2;
        land_stage_time_ = now;
      }
      else if (land_stage_ == 2 && !state.armed)
      {
        finishMission();
      }
      else if (land_stage_ == 2 && now - phase_start_ > ros::Duration(45.0) && !land_warned_)
      {
        land_warned_ = true;
        ROS_WARN("[mission] still armed 45 s after AUTO.LAND; check the landing");
      }
      break;
    }

    case Phase::DONE:
    {
      control_mode_ = ControlMode::IDLE;
      break;
    }
  }
}

void MoveBaseMission::beginTakeoff()
{
  phase_ = Phase::TAKEOFF;
  phase_start_ = ros::Time::now();
  control_mode_ = ControlMode::HOLD;
  target_position_.x = 0.0;
  target_position_.y = 0.0;
  target_position_.z = fly_height_;
  setStatus("TAKEOFF");
}

void MoveBaseMission::startTask(Task task)
{
  task_ = task;
  leg_ = 0;
  goal_active_ = false;
  goal_attempts_ = 0;
  hits_.clear();
  planner_velxy_posz_ = geometry_msgs::Point();
  planner_velxy_posz_.z = fly_height_;

  if (currentRoute().empty())
  {
    ROS_WARN("[mission] %s route is empty; skipping",
             task == Task::HAZARD ? "hazard" : (task == Task::PERSON ? "person" : "return"));
    if (task == Task::HAZARD)
      startTask(Task::PERSON);
    else if (task == Task::PERSON)
      startTask(Task::RETURN);
    else
      beginLand();
    return;
  }

  control_mode_ = ControlMode::TRACK;
  phase_ = Phase::TRANSIT;
  phase_start_ = ros::Time::now();
  setStatus(task == Task::HAZARD ? "TASK_HAZARD"
                                 : (task == Task::PERSON ? "TASK_PERSON" : "TASK_RETURN"));
}

void MoveBaseMission::nextLeg()
{
  ++leg_;
  goal_active_ = false;
  goal_attempts_ = 0;
  if (leg_ >= currentRoute().size())
  {
    if (task_ == Task::HAZARD)
    {
      const geometry_msgs::PoseStamped pose = poseSnapshot();
      ROS_WARN("[mission] no matching hazard at any card; dropping fire material at the last "
               "card (%.2f, %.2f)",
               pose.pose.position.x, pose.pose.position.y);
      after_drop_task_ = Task::PERSON;
      control_mode_ = ControlMode::HOLD;
      beginDrop(pose.pose.position, true);
      return;
    }
    if (task_ == Task::PERSON)
    {
      ROS_WARN("[mission] no injured person found in the rescue zone");
      startTask(Task::RETURN);
      return;
    }
    beginLand();
    return;
  }
  phase_ = Phase::TRANSIT;
  phase_start_ = ros::Time::now();
  // 回到跟规划器飞。**必须在这里恢复**：上一个点在到达那一下调过 holdAt()，
  // 它把 control_mode_ 设成 HOLD；不恢复的话 TRANSIT 阶段照常发 goal、照常等
  // move_base 的结果，但 50 Hz 控制环一直在原地保持位置 —— 飞机停在第一个航点上
  // 不动，move_base 的 goal 永远不成功，25 s 超时重发一次、再 25 s 才换下一个点。
  // 现象就是"只飞一个航点然后一直悬停"（2026-10-24 的日志）。
  control_mode_ = ControlMode::TRACK;
}

void MoveBaseMission::onArrived()
{
  const std::vector<Waypoint> &route = currentRoute();
  if (leg_ >= route.size())
  {
    nextLeg();
    return;
  }
  if (route[leg_].detect)
  {
    beginSettle(route[leg_]);
  }
  else
  {
    nextLeg();
  }
}

void MoveBaseMission::beginSettle(const Waypoint &waypoint)
{
  current_waypoint_ = waypoint;
  phase_ = Phase::SETTLE;
  phase_start_ = ros::Time::now();
  control_mode_ = ControlMode::HOLD;
  holdCurrent();
  target_position_.z = detect_height_;
  hits_.clear();
  last_infer_ = ros::Time(0);

  int index = 1;
  int total = 0;
  if (task_ == Task::HAZARD)
  {
    total = hazard_detect_num_;
    index = static_cast<int>(leg_) - (static_cast<int>(hazard_route_.size()) - hazard_detect_num_) + 1;
  }
  else
  {
    total = person_detect_num_;
    index = static_cast<int>(leg_) - (static_cast<int>(person_route_.size()) - person_detect_num_) + 1;
  }
  std::ostringstream status;
  status << "SETTLE " << current_waypoint_.name << " (" << index << "/" << total << ")";
  setStatus(status.str());
}

void MoveBaseMission::beginDetect()
{
  phase_ = Phase::DETECT;
  phase_start_ = ros::Time::now();
  last_infer_ = ros::Time(0);
  hits_.clear();
  std::string target;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target = (task_ == Task::HAZARD) ? target_hazard_ : target_person_;
  }
  setStatus("DETECT " + current_waypoint_.name + " target=" + target);
}

bool MoveBaseMission::refreshDetectTarget(Detect::Detection &best, cv::Point2f &body_offset)
{
  std::string target;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target = (task_ == Task::HAZARD) ? target_hazard_ : target_person_;
  }
  if (target == "any" || target == "none" || target.empty())
    target.clear();

  const geometry_msgs::PoseStamped pose = poseSnapshot();
  const double height = std::max(0.2, pose.pose.position.z - 0.02);
  const std::vector<Detect::Detection> detections = detect_.infer();

  bool found = false;
  double best_distance = max_center_offset_;
  for (const auto &detection : detections)
  {
    if (!target.empty() && detection.label != target)
      continue;
    cv::Point2f body;
    if (!detect_.pixelToBody(detection.center, height, body))
      continue;
    const double distance = std::hypot(body.x, body.y);
    if (distance <= max_center_offset_ && distance < best_distance)
    {
      best = detection;
      body_offset = body;
      best_distance = distance;
      found = true;
    }
  }

  if (!found && ros::Time::now() - last_detect_log_ > ros::Duration(2.0))
  {
    last_detect_log_ = ros::Time::now();
    std::ostringstream labels;
    for (size_t i = 0; i < detections.size() && i < 3; ++i)
      labels << detections[i].label << ":" << static_cast<int>(detections[i].confidence * 100)
             << "% ";
    ROS_INFO("[mission] looking for '%s': %zu detection(s) %s", target.c_str(), detections.size(),
             labels.str().c_str());
  }
  return found;
}

void MoveBaseMission::onDetectFailed()
{
  ROS_WARN("[mission] %s not found at %s within %.1f s", task_ == Task::HAZARD ? "hazard"
                                                                    : "person",
           current_waypoint_.name.c_str(), detect_timeout_);
  nextLeg();
}

void MoveBaseMission::onConfirmed(const Detect::Detection &detection, const cv::Point2f &body_offset)
{
  const bool fire = (task_ == Task::HAZARD);
  const geometry_msgs::PoseStamped pose = poseSnapshot();

  detect_.annotate(detection, fire ? kRed : kBlue, fire ? "hazard" : "person", annotate_hold_);

  std::ostringstream status;
  status << (fire ? "HAZARD_FOUND " : "PERSON_FOUND ") << detection.label << " "
         << static_cast<int>(detection.confidence * 100) << "%";
  setStatus(status.str());
  ROS_INFO("[mission] confirmed %s %.2f at offset (%.2f, %.2f) m", detection.label.c_str(),
           detection.confidence, body_offset.x, body_offset.y);

  if (!fire && !rescue_drop_)
  {
    ROS_INFO("[mission] stage 1: person marked, no rescue drop");
    startTask(Task::RETURN);
    return;
  }

  geometry_msgs::Point target = pose.pose.position;
  bool aligned = false;
  if (!fire && rescue_drop_field_set_)
  {
    const FieldPoint map = fieldToMap(rescue_drop_field_);
    target.x = map.x;
    target.y = map.y;
    aligned = true;
    ROS_INFO("[mission] rescue drop uses the drawn position (%.2f, %.2f) field -> (%.2f, %.2f) map",
             rescue_drop_field_.x, rescue_drop_field_.y, target.x, target.y);
  }
  else if (drop_align_mode_ != "fixed")
  {
    cv::Point2f offset = body_offset;
    const double norm = std::hypot(offset.x, offset.y);
    if (norm > max_align_offset_)
    {
      ROS_WARN("[mission] vision offset %.2f m exceeds max_align_offset %.2f; dropping at the "
               "waypoint",
               norm, max_align_offset_);
      offset = cv::Point2f(0.0f, 0.0f);
    }
    if (std::hypot(offset.x, offset.y) > 0.02)
    {
      const geometry_msgs::Point delta = bodyToLocal(offset);
      target.x += delta.x;
      target.y += delta.y;
      aligned = true;
    }
  }
  if (!aligned)
    ROS_INFO("[mission] dropping at the current waypoint (no vision offset applied)");

  target.z = drop_height_;
  after_drop_task_ = fire ? Task::PERSON : Task::RETURN;
  beginDrop(target, fire);
}

void MoveBaseMission::beginDrop(const geometry_msgs::Point &target, bool fire)
{
  drop_is_fire_ = fire;
  drop_target_ = target;
  drop_target_.z = drop_height_;
  phase_ = Phase::DESCEND;
  phase_start_ = ros::Time::now();
  control_mode_ = ControlMode::HOLD;
  target_position_ = drop_target_;
  drop_sent_ = false;
  ack_received_ = false;
  setStatus(fire ? "DESCEND_FIRE" : "DESCEND_RESCUE");
}

void MoveBaseMission::beginClimb()
{
  phase_ = Phase::CLIMB;
  phase_start_ = ros::Time::now();
  control_mode_ = ControlMode::HOLD;
  target_position_ = drop_target_;
  target_position_.z = fly_height_;
  setStatus("CLIMB");
}

void MoveBaseMission::beginLand()
{
  phase_ = Phase::LAND;
  phase_start_ = ros::Time::now();
  land_stage_ = 0;
  land_stage_time_ = ros::Time::now();
  land_warned_ = false;
  control_mode_ = ControlMode::HOLD;
  holdCurrent();
  sendGpio("CAMERA_FRONT");
  setStatus("LAND");
  ROS_INFO("[mission] camera to front, then POSCTL -> AUTO.LAND");
}

void MoveBaseMission::finishMission()
{
  phase_ = Phase::DONE;
  control_mode_ = ControlMode::IDLE;
  setStatus("MISSION_COMPLETE");
  ROS_INFO("[mission] disarmed; mission complete. Stop the timer.");
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

std::string MoveBaseMission::resolveHazardTarget(const std::string &text) const
{
  // 去掉首尾空白，先按 labels.txt 的类名精确匹配。
  std::string::size_type begin = 0;
  std::string::size_type end = text.size();
  while (begin < end && std::isspace(static_cast<unsigned char>(text[begin])))
    ++begin;
  while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])))
    --end;
  const std::string trimmed = text.substr(begin, end - begin);
  if (trimmed.empty())
    return "";

  if (detect_.hasLabel(trimmed))
    return trimmed;

  std::string lower = trimmed;
  for (auto &character : lower)
    character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  if (detect_.hasLabel(lower))
    return lower;

  // 赛题给的是中文危险品名称；这里把常见叫法映射到 labels.txt 的类名。
  // 新模型（YOLOv5n，4 类）只有 explosive / flammable / toxic / person，
  // 其余危险品类别不再参与识别，所以别名表也只保留这四类。
  static const std::map<std::string, std::string> aliases = {
      {"hazard_flammable", "flammable"}, {"flammable", "flammable"},
      {"火灾", "flammable"},             {"易燃", "flammable"},
      {"1-02", "flammable"},
      {"hazard_explosive", "explosive"}, {"explosive", "explosive"},
      {"爆炸", "explosive"},             {"易爆", "explosive"},
      {"1-03", "explosive"},
      {"hazard_toxic", "toxic"},         {"toxic", "toxic"},
      {"有毒", "toxic"},                 {"中毒", "toxic"},
      {"1-05", "toxic"},
  };

  auto it = aliases.find(lower);
  if (it != aliases.end())
    return it->second;
  it = aliases.find(trimmed);
  if (it != aliases.end())
    return it->second;
  return "";
}

void MoveBaseMission::sendGoal(const Waypoint &waypoint)
{
  if (!move_base_ready_)
    return;
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = waypoint.map_x;
  goal.target_pose.pose.position.y = waypoint.map_y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.z = std::sin(hold_yaw_ * 0.5);
  goal.target_pose.pose.orientation.w = std::cos(hold_yaw_ * 0.5);

  move_base_.sendGoal(goal);
  // 发 goal 就意味着接下来要跟规划器飞。控制环的模式只在这里和 holdAt() 里改，
  // 到达/识别前的那次 holdAt() 会把它设成 HOLD，所以每次发 goal 都恢复 TRACK ——
  // 否则 goal 照发、飞机却一直原地保持（"只飞一个航点然后悬停"的那个 bug）。
  control_mode_ = ControlMode::TRACK;
  goal_active_ = true;
  goal_sent_ = ros::Time::now();
  ++goal_attempts_;
  ROS_INFO("[mission] goal %s: field (%.2f, %.2f) -> map (%.2f, %.2f)", waypoint.name.c_str(),
           waypoint.field.x, waypoint.field.y, waypoint.map_x, waypoint.map_y);
}

void MoveBaseMission::holdAt(const geometry_msgs::Point &point)
{
  control_mode_ = ControlMode::HOLD;
  target_position_ = point;
}

void MoveBaseMission::holdCurrent()
{
  const geometry_msgs::PoseStamped pose = poseSnapshot();
  holdAt(pose.pose.position);
}

void MoveBaseMission::sendGpio(const std::string &command)
{
  std_msgs::String message;
  message.data = command;
  gpio_cmd_pub_.publish(message);
  ROS_INFO("[mission] gpio: %s", command.c_str());
}

void MoveBaseMission::setStatus(const std::string &status)
{
  std_msgs::String message;
  message.data = status;
  status_pub_.publish(message);
  ROS_INFO("[mission] status: %s", status.c_str());
}

void MoveBaseMission::requestMode(const std::string &mode)
{
  ROS_INFO("[mission] request mode %s", mode.c_str());
  flightControl.set_mode(mode);
}

bool MoveBaseMission::atHeight(double z, double tolerance) const
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return std::fabs(current_pose_.pose.position.z - z) < tolerance;
}

double MoveBaseMission::yaw() const
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return yawOf(current_pose_.pose.orientation);
}

geometry_msgs::PoseStamped MoveBaseMission::poseSnapshot() const
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return current_pose_;
}

mavros_msgs::State MoveBaseMission::stateSnapshot() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return fcu_state_;
}

geometry_msgs::Point MoveBaseMission::bodyToLocal(const cv::Point2f &body) const
{
  const double heading = yaw();
  geometry_msgs::Point out;
  out.x = std::cos(heading) * body.x - std::sin(heading) * body.y;
  out.y = std::sin(heading) * body.x + std::cos(heading) * body.y;
  out.z = 0.0;
  return out;
}

MoveBaseMission::FieldPoint MoveBaseMission::fieldToMap(const FieldPoint &field) const
{
  const FieldPoint clamped = clampField(field);
  const double angle = -field_yaw_deg_ * M_PI / 180.0;
  FieldPoint map;
  map.x = std::cos(angle) * clamped.x - std::sin(angle) * clamped.y;
  map.y = std::sin(angle) * clamped.x + std::cos(angle) * clamped.y;
  return map;
}

MoveBaseMission::FieldPoint MoveBaseMission::clampField(const FieldPoint &field) const
{
  FieldPoint out = field;
  if (!enforce_safe_field_)
    return out;
  out.x = clamp(out.x, field_min_x_, field_max_x_);
  out.y = clamp(out.y, field_min_y_, field_max_y_);
  if (out.x != field.x || out.y != field.y)
    ROS_WARN("[mission] field point (%.2f, %.2f) outside the safe field; clamped to (%.2f, %.2f)",
             field.x, field.y, out.x, out.y);
  return out;
}

const std::vector<MoveBaseMission::Waypoint> &MoveBaseMission::currentRoute() const
{
  switch (task_)
  {
    case Task::HAZARD:
      return hazard_route_;
    case Task::PERSON:
      return person_route_;
    case Task::RETURN:
    default:
      return return_route_;
  }
}

double MoveBaseMission::yawOf(const geometry_msgs::Quaternion &q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void MoveBaseMission::logRoute(const std::vector<Waypoint> &route, const char *name, double yaw_deg)
{
  if (route.empty())
  {
    ROS_WARN("[mission] %s route is empty", name);
    return;
  }
  ROS_INFO("[mission] %s route (%zu points, field_yaw=%.1f deg):", name, route.size(), yaw_deg);
  for (size_t i = 0; i < route.size(); ++i)
  {
    ROS_INFO("[mission]   %s[%zu] field=(%6.2f,%6.2f) map=(%6.2f,%6.2f)%s", name, i,
             route[i].field.x, route[i].field.y, route[i].map_x, route[i].map_y,
             route[i].detect ? "  <- detect" : "");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_with_waypoint");
  ros::NodeHandle nh_private("~");
  MoveBaseMission mission(nh_private, argc, argv);
  ros::MultiThreadedSpinner spinner(6);
  spinner.spin();
  return 0;
}
