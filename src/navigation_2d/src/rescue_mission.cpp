#include "mission_with_move_base.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <XmlRpcValue.h>
#include <tf/transform_datatypes.h>

namespace
{
// Local definition rather than M_PI: M_PI is not guaranteed by the C++
// standard (it is a POSIX/GNU extension) and this file must build unchanged on
// the ROS noetic toolchain.
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

double xmlNumber(const XmlRpc::XmlRpcValue &value, const char *key, double fallback)
{
  if (!value.hasMember(key))
  {
    return fallback;
  }
  const XmlRpc::XmlRpcValue &member = value[key];
  if (member.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    return static_cast<int>(member);
  }
  if (member.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    return static_cast<double>(member);
  }
  return fallback;
}

float clampFloat(float value, float low, float high)
{
  return std::max(low, std::min(value, high));
}
}

RescueMission::RescueMission(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : nh_(nh), private_nh_(private_nh), flight_control_(""), vision_(nh_, private_nh_)
{
  private_nh_.param<float>("fly_height", fly_height_, 1.5f);
  private_nh_.param<float>("task_height", task_height_, 1.2f);
  // Fixed mission heading in degrees (odom frame, 0 = nose along +x). See the
  // comment on mission_heading_ in the header: this replaces "yaw sampled at
  // takeoff" with an explicit mission constant.
  private_nh_.param<float>("fix_heading_deg", fix_heading_deg_, 0.0f);
  // Heading of the airframe in the FIELD frame at the moment localization
  // started. See the note on field_yaw_deg_ in the header: this is the only
  // thing that reconciles the field-frame route tables with the odom-frame
  // goals, and it is inert at its default of 0.0.
  private_nh_.param<float>("field_yaw_deg", field_yaw_deg_, 0.0f);
  // Normalise into (-180, 180]; a value outside that range is a typo rather
  // than a heading, and it would otherwise rotate the route by whole turns.
  while (field_yaw_deg_ > 180.0f)
  {
    field_yaw_deg_ -= 360.0f;
  }
  while (field_yaw_deg_ <= -180.0f)
  {
    field_yaw_deg_ += 360.0f;
  }
  field_yaw_rad_ = static_cast<double>(field_yaw_deg_) * kDegToRad;
  private_nh_.param<float>("xy_gain", xy_gain_, 0.8f);
  private_nh_.param<float>("z_gain", z_gain_, 1.0f);
  private_nh_.param<float>("max_velocity", max_velocity_, 0.35f);
  private_nh_.param<float>("position_tolerance", position_tolerance_, 0.12f);
  private_nh_.param<float>("altitude_tolerance", altitude_tolerance_, 0.08f);
  private_nh_.param<float>("route_altitude_tolerance", route_altitude_tolerance_, 0.40f);
  private_nh_.param<float>("task_alignment_tolerance", task_alignment_tolerance_, 0.08f);
  private_nh_.param<float>("camera_fx", camera_fx_, 874.0f);
  private_nh_.param<float>("camera_fy", camera_fy_, 874.0f);
  private_nh_.param<float>("pixel_to_body_x_sign", pixel_to_body_x_sign_, 1.0f);
  private_nh_.param<float>("pixel_to_body_y_sign", pixel_to_body_y_sign_, 1.0f);
  // 0.15, not 0.45: see the note on the member. The launch file carries the
  // same number; tools/check_config.py checks both against the wall clearance.
  private_nh_.param<float>("max_task_correction", max_task_correction_, 0.15f);
  private_nh_.param<float>("align_timeout", align_timeout_, 5.0f);
  private_nh_.param<float>("align_body_clearance", align_body_clearance_, 0.35f);
  private_nh_.param<float>("waypoint_dwell", waypoint_dwell_, 0.6f);
  private_nh_.param<float>("camera_settle_time", camera_settle_time_, 0.8f);
  // Silence threshold for the task-phase detection wait. Before this existed the
  // only line was "no stable detection after 5.0 s", which reads the same whether
  // the detector disagreed with the mission or never spoke; on 2026-10-18 it was
  // the latter (a self-deadlock in MissionVision) and the log gave no way to tell.
  private_nh_.param<float>("vision_silence_timeout", vision_silence_timeout_, 2.0f);
  private_nh_.param<float>("task_timeout", task_timeout_, 12.0f);
  private_nh_.param<float>("task_alignment_dwell", task_alignment_dwell_, 0.7f);
  private_nh_.param<float>("drop_settle_time", drop_settle_time_, 1.5f);
  private_nh_.param<float>("navigation_timeout", navigation_timeout_, 30.0f);
  private_nh_.param<float>("planner_velocity_timeout", planner_velocity_timeout_, 0.6f);
  // How long a waypoint may go without useful planner motion before the node
  // declares the planner stalled and retries (then aborts). DWA reports
  // "failed to produce path" and then publishes zero velocity while it
  // recovers, and during that time planner_motion_received_at_ is never
  // refreshed, so this timer is what decides whether the mission survives a
  // transient local-planning failure. 5.0 s was tight enough that a single
  // "failed to produce path" at 67.1 s ran out the clock and aborted at 72.9 s
  // before either retry could succeed.
  private_nh_.param<float>("planner_failure_abort_timeout", planner_failure_abort_timeout_, 10.0f);
  // See the note on route_no_motion_timeout_ in the header. This one fires
  // before planner_failure_abort_timeout_ on purpose and does NOT retry: a leg
  // that has never received a moving command is not a transient planner
  // hiccup, and resending the same unreachable goal just buys 10 more seconds
  // of hover at 1.5 m.
  private_nh_.param<float>("route_no_motion_timeout", route_no_motion_timeout_, 6.0f);
  // Pre-arm gate on the vision backend. See the note on require_vision_ready_
  // in the header; the timeout budget is the Jetson's engine deserialisation
  // (26.5 s measured) plus the camera pipeline coming up.
  private_nh_.param<bool>("require_vision_ready", require_vision_ready_, true);
  private_nh_.param<float>("vision_ready_timeout", vision_ready_timeout_, 60.0f);
  private_nh_.param<float>("camera_command_timeout", camera_command_timeout_, 5.0f);
  private_nh_.param<float>("camera_command_retry_interval", camera_command_retry_interval_, 1.5f);
  private_nh_.param<float>("landing_camera_post_move_settle", landing_camera_post_move_settle_, 0.3f);
  private_nh_.param<float>("camera_down_post_move_settle", camera_down_post_move_settle_, 0.3f);
  private_nh_.param<float>("landing_timeout", landing_timeout_, 20.0f);
  private_nh_.param<float>("safe_field_min_x", safe_field_min_x_, -0.45f);
  private_nh_.param<float>("safe_field_max_x", safe_field_max_x_, 3.15f);
  private_nh_.param<float>("safe_field_min_y", safe_field_min_y_, -3.15f);
  private_nh_.param<float>("safe_field_max_y", safe_field_max_y_, 0.45f);
  private_nh_.param<float>("abort_land_home_radius", abort_land_home_radius_, 0.25f);
  private_nh_.param<float>("abort_land_settle", abort_land_settle_, 0.5f);
  private_nh_.param<float>("abort_land_retry_interval", abort_land_retry_interval_, 2.0f);
  private_nh_.param<int>("max_navigation_retries", max_navigation_retries_, 2);
  private_nh_.param<bool>("auto_start", auto_start_, true);

  // ---- competition stage, and the drawn round-2 drop position --------------
  //
  // Round 1: the fire material is dropped at the 危险品任务区, the aircraft then
  // flies over every 人员救助任务区 and returns home. Round 2: the same flight,
  // plus the rescue material, dropped at a position that is drawn by lot on the
  // day and therefore cannot live in the route table.
  //
  // Two independent inputs describe the same fact, and they are reconciled here
  // rather than in the launch file so that neither "stage:=2 with the old flag
  // left at false" nor "rescue_drop_enabled:=true re-used from a round-2 launch"
  // can silently produce a flight with the wrong number of drops. The node is
  // the single deciding place; the launch files only pass arguments through.
  private_nh_.param<int>("stage", stage_, 1);
  if (stage_ != 1 && stage_ != 2)
  {
    ROS_WARN("stage %d is neither 1 nor 2; flying this run as stage 1", stage_);
    stage_ = 1;
  }
  private_nh_.param<bool>("rescue_drop_enabled", rescue_drop_enabled_, false);
  if (stage_ >= 2)
  {
    rescue_drop_enabled_ = true;
  }
  else if (rescue_drop_enabled_)
  {
    ROS_WARN("rescue_drop_enabled=true but stage=%d; the rescue drop only exists "
             "in round 2, so this run is treated as stage 2", stage_);
    stage_ = 2;
  }

  // There is no safe default for the drawn position: (0, 0) is the take-off pad,
  // which is inside the field and would be accepted by every check except the
  // one that matters. So the parameter is required, and what "not supplied"
  // looks like is a point outside the enclosure - the launch file's 999.0
  // default included - which turns into a refusal to take off (missionCallback)
  // instead of a drop somewhere arbitrary.
  double rescue_drop_x = 0.0;
  double rescue_drop_y = 0.0;
  private_nh_.param<double>("rescue_drop_x", rescue_drop_x, 1.0e9);
  private_nh_.param<double>("rescue_drop_y", rescue_drop_y, 1.0e9);
  const bool drop_finite = std::isfinite(rescue_drop_x) && std::isfinite(rescue_drop_y);
  if (drop_finite &&
      rescue_drop_x >= safe_field_min_x_ && rescue_drop_x <= safe_field_max_x_ &&
      rescue_drop_y >= safe_field_min_y_ && rescue_drop_y <= safe_field_max_y_)
  {
    rescue_drop_point_.x = static_cast<float>(rescue_drop_x);
    rescue_drop_point_.y = static_cast<float>(rescue_drop_y);
    rescue_drop_point_.z = fly_height_;
    have_rescue_drop_point_ = true;
    rescue_drop_route_.clear();
    rescue_drop_route_.push_back(rescue_drop_point_);
  }

  // ---- how the drop point is decided --------------------------------------
  //
  // This is the one decision in this file that was made by weighing three
  // options instead of by fixing a failure, so the weighing is written down.
  //
  // The task zones are 0.45 m frames drawn around 0.40 m cards, i.e. the payload
  // has to land inside roughly +/-0.22 m of the drawn centre. Two independent
  // estimates of where that centre is exist:
  //
  //   * the route table - an odom coordinate authored off the drawing and read
  //     by the mission's own arrival test at position_tolerance. It is a claim
  //     about the world, and its error is whatever PX4's local position estimate
  //     is off by. Nothing in the loop measures that error.
  //   * the camera (mission_vision's yellow/blue candidates and red/blue frame)
  //     measures the card itself and therefore closes the loop on the real
  //     error - it is the only signal here that is about the target rather than
  //     about the vehicle. It costs a hover, and it can fail outright: 5 stable
  //     frames are required at 15 fps, and the 3 m fence, a shadow or a JPEG
  //     artefact can each suppress the detection.
  //
  // Camera-only is what this node used to do, and it turned that failure mode
  // into the end of the run: "Hazard target detection timeout" aborts the
  // mission, which is the worst possible trade for one missed blob when a
  // waypoint drop would still have scored.
  // Waypoint-only removes the failure mode and the accuracy together: the
  // payload lands wherever the position estimate says, with no second opinion.
  // Camera-then-waypoint keeps the camera's accuracy and the waypoint's
  // availability, and it is the default. It only falls back when the vehicle is
  // ALREADY within drop_waypoint_tolerance_ of the authored waypoint, so a
  // detector failure can never become "drop somewhere random" - the failure
  // modes are "drop as well as the position estimate allows" or "hold and
  // report", never "drop in the wrong place".
  //
  // Round 1 should fly the default. The other two modes exist because the venue
  // is not this simulation: if the camera misbehaves on the day,
  // drop_align_mode:=waypoint still completes the flight, and if the field is
  // trusted more than the detector, drop_align_mode:=vision restores the strict
  // behaviour.
  std::string drop_align_mode = "vision_with_fallback";
  private_nh_.param<std::string>("drop_align_mode", drop_align_mode,
                                 std::string("vision_with_fallback"));
  if (drop_align_mode == "vision")
  {
    drop_align_mode_ = DROP_ALIGN_VISION;
  }
  else if (drop_align_mode == "waypoint")
  {
    drop_align_mode_ = DROP_ALIGN_WAYPOINT;
  }
  else if (drop_align_mode == "vision_with_fallback")
  {
    drop_align_mode_ = DROP_ALIGN_VISION_FALLBACK;
  }
  else
  {
    ROS_WARN("drop_align_mode '%s' is not 'vision', 'waypoint' or "
             "'vision_with_fallback'; using vision_with_fallback",
             drop_align_mode.c_str());
    drop_align_mode = "vision_with_fallback";
  }
  private_nh_.param<float>("drop_vision_timeout", drop_vision_timeout_, 5.0f);
  private_nh_.param<float>("drop_waypoint_tolerance", drop_waypoint_tolerance_, 0.20f);
  if (drop_vision_timeout_ >= task_timeout_)
  {
    ROS_WARN("drop_vision_timeout=%.1fs is not shorter than task_timeout=%.1fs; "
             "the fallback would never get a chance before the abort",
             drop_vision_timeout_, task_timeout_);
  }
  private_nh_.param<bool>("use_position_hint", use_position_hint_, true);
  private_nh_.param<bool>("abort_on_offboard_loss", abort_on_offboard_loss_, true);
  private_nh_.param<bool>("enforce_safe_field", enforce_safe_field_, false);
  private_nh_.param<bool>("require_servo_ready", require_servo_ready_, true);
  // /cmd_vel 的帧约定只有一种：odom（= plan 帧）。
  //
  // 2026-10-17 删掉了另一条路。以前这里是 "body"/"odom" 两选一，与
  // ego_trajectory_controller 的 output_frame 配对；"body" 那条会用
  // current_yaw_（PX4 EKF）把跟随器用 odom->base_link 的 yaw 转进机体的速度
  // 再转回来，两个来自不同估计器的角度相减，残差就是一个横向推力 —— 而
  // mission 的位置环看不见它（它把 setpoint 钉在飞机自己的实测位置上）。
  // 只剩 odom 之后，整条链里恰好只有一次旋转，这类失配不可能再出现。
  ROS_INFO("cmd_vel is read as an odom-frame velocity (single rotation in the "
           "whole chain, no body->world conversion here)");

  loadRoute("hazard_route", hazard_route_);
  loadRoute("person_route", person_route_);
  loadRoute("return_route", return_route_);
  applyRouteFrameYaw();

  // Deliberately NOT /move_base_simple/goal.
  //
  // mavros_extras' guided_target plugin ADVERTISES that exact topic: every
  // time the FCU streams a POSITION_TARGET_GLOBAL_INT (it does so at ~10 Hz in
  // any position-control mode) the plugin converts it and re-publishes it as a
  // geometry_msgs/PoseStamped stamped "map", with z hard-zeroed. That collided
  // head-on with the waypoint stream: EgoPlanner2DNode::goalCallback rejects
  // any goal whose frame_id differs from its global_frame ("odom") by calling
  // clearPlan(), so a foreign publisher sitting on the shared topic could wipe
  // the plan of a leg that was already being flown - a remote freeze with no
  // error anywhere.
  //
  // The name collision was the whole bug, so it is removed at the source: the
  // mission owns /mission/route_goal, and navigation_2d.launch remaps EGO's
  // "goal" input onto it. Nobody else may publish there. Consequences of the
  // rename:
  //   * RViz's "2D Nav Goal" and Foxglove's publish-point no longer drive the
  //     planner. That is intended - they were the other way a "map"-frame goal
  //     reached EGO. Use the mission, or publish on /mission/route_goal in the
  //     odom frame by hand.
  //   * The frame check in goalCallback is now a real invariant rather than a
  //     load-bearing safety net, which is why it may keep clearing the plan
  //     on mismatch: the only thing that can produce a mismatch now is this
  //     node.
  //
  // Killing the plugin (mavros px4_pluginlists.yaml -> plugin_denylist:
  // [guided_target]) also stops the "PositionTargetGlobal failed because no
  // origin" spam, but it lives outside this repository, so the rename is what
  // actually has to land here.
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mission/route_goal", 1);
  goal_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
  gpio_pub_ = nh_.advertise<std_msgs::String>("/gpio_cmd", 1);
  vision_mode_pub_ = nh_.advertise<std_msgs::String>("/mission/vision/mode", 1);
  // Latched, so a planner that starts later still learns the current state.
  planning_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/trigger_plan", 1, true);

  state_sub_ = nh_.subscribe("/mavros/state", 1, &RescueMission::stateCallback, this,
                             ros::TransportHints().tcpNoDelay());
  servo_ready_sub_ = nh_.subscribe("/servo_ready", 1,
                                   &RescueMission::servoReadyCallback, this);
  pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1,
                            &RescueMission::poseCallback, this,
                            ros::TransportHints().tcpNoDelay());
  planner_velocity_sub_ = nh_.subscribe("/cmd_vel", 1,
                                        &RescueMission::plannerVelocityCallback, this,
                                        ros::TransportHints().tcpNoDelay());
  gpio_ack_sub_ = nh_.subscribe("/gpio_ack", 1, &RescueMission::gpioAckCallback, this);
  detection_sub_ = nh_.subscribe("/mission/vision/detection", 1,
                                 &RescueMission::detectionCallback, this);

  control_timer_ = nh_.createTimer(ros::Duration(0.02), &RescueMission::controlCallback, this);
  mission_timer_ = nh_.createTimer(ros::Duration(0.1), &RescueMission::missionCallback, this);

  target_position_.z = 0.0;
  ROS_INFO("Rescue mission initialized. auto_start=%s planner_timeout=%.2fs",
           auto_start_ ? "true" : "false",
           planner_velocity_timeout_);
  // The takeoff gate, spelled out. This line is the answer to "does auto_start
  // wait for the detector?" - before 2026-10-17 the answer was no and the log
  // did not say so.
  ROS_INFO("Pre-arm gate: servo=%s vision=%s (vision needs a detection message "
           "within 2.0 s of arming, timeout %.0fs) route_no_motion=%.1fs",
           require_servo_ready_ ? "required" : "not required",
           require_vision_ready_ ? "required" : "NOT required",
           static_cast<double>(vision_ready_timeout_),
           static_cast<double>(route_no_motion_timeout_));
  // One block that answers "which round is this, and where does it drop" from a
  // log paste alone, because both answers are set by hand at the venue.
  ROS_INFO("Competition stage %d: %s", stage_,
           stage_ >= 2 ? "hazard drop + rescue-material drop"
                       : "hazard drop only, no rescue-material drop");
  ROS_INFO("Drop alignment mode: %s (%s)", dropAlignModeName(drop_align_mode_),
           drop_align_mode_ == DROP_ALIGN_VISION
               ? "camera only, a detection timeout aborts"
               : drop_align_mode_ == DROP_ALIGN_WAYPOINT
                     ? "authored waypoint only, camera not used"
                     : "camera first, authored waypoint as the fallback");
  if (drop_align_mode_ != DROP_ALIGN_WAYPOINT)
  {
    ROS_INFO("Drop fallback: after %.1fs without a stable detection, drop on the "
             "authored waypoint if the vehicle is within %.2f m of it",
             drop_vision_timeout_, drop_waypoint_tolerance_);
  }
  if (rescue_drop_enabled_)
  {
    if (have_rescue_drop_point_)
    {
      ROS_INFO("Rescue-material drop waypoint: (%.2f, %.2f) in odom, flown after "
               "every person task area", rescue_drop_point_.x, rescue_drop_point_.y);
    }
    else
    {
      ROS_FATAL("stage %d needs rescue_drop_x and rescue_drop_y inside the field "
                "(x %.2f..%.2f, y %.2f..%.2f); got (%.2f, %.2f). Refusing to take "
                "off - set both on the command line to the drawn position.",
                stage_, safe_field_min_x_, safe_field_max_x_,
                safe_field_min_y_, safe_field_max_y_, rescue_drop_x, rescue_drop_y);
    }
  }
  // Establish the latched state as "do not plan" before anything else happens.
  // Planning is switched on only for the duration of an actual route (see
  // startRoute / enterState), so a planner that has nothing to fly stops
  // publishing trajectories instead of replanning forever.
  setPlanningEnabled(false);
}

RescueMission::~RescueMission()
{
  // Best effort only, and worth being honest about its reach: in the normal
  // Ctrl-C path ros::spin() has already returned, so ros::ok() is false and this
  // publish does not happen - which is fine, because Ctrl-C takes the planner
  // down with it. What this does cover is the object being destroyed while ROS is
  // still running.
  //
  // It does NOT cover the case that would actually hurt: the mission node dying
  // while the planner keeps running during a route. Nothing in this design
  // notices that, because the planner has no heartbeat to lose. If that ever
  // shows up as a symptom, the fix belongs in EgoPlanner2DNode (a goal/mission
  // watchdog), not here.
  if (planning_enabled_ && ros::ok())
  {
    std_msgs::Bool message;
    message.data = false;
    planning_trigger_pub_.publish(message);
    planning_enabled_ = false;
  }
}

void RescueMission::loadRoute(const std::string &name,
                              std::vector<geometry_msgs::Point> &route)
{
  XmlRpc::XmlRpcValue value;
  if (!private_nh_.getParam(name, value))
  {
    ROS_WARN("Route '%s' is not configured", name.c_str());
    return;
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Route '%s' must be an array", name.c_str());
    return;
  }

  for (int i = 0; i < value.size(); ++i)
  {
    if (value[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("Ignoring invalid point %d in route '%s'", i, name.c_str());
      continue;
    }
    geometry_msgs::Point point;
    point.x = xmlNumber(value[i], "x", 0.0);
    point.y = xmlNumber(value[i], "y", 0.0);
    point.z = xmlNumber(value[i], "z", fly_height_);
    route.push_back(point);
  }

  ROS_INFO("Loaded route '%s' with %zu points", name.c_str(), route.size());
}

geometry_msgs::Point RescueMission::fieldToOdom(const geometry_msgs::Point &field) const
{
  // odom = R(-psi) * field. The field's +x axis sits at -psi in odom, so a
  // field-frame number has to be rotated back by -psi to become the odom
  // coordinate that puts the vehicle over the point it names.
  const double c = std::cos(field_yaw_rad_);
  const double s = std::sin(field_yaw_rad_);
  geometry_msgs::Point out;
  out.x = c * field.x + s * field.y;
  out.y = -s * field.x + c * field.y;
  out.z = field.z;
  return out;
}

geometry_msgs::Point RescueMission::odomToField(const geometry_msgs::Point &odom) const
{
  const double c = std::cos(field_yaw_rad_);
  const double s = std::sin(field_yaw_rad_);
  geometry_msgs::Point out;
  out.x = c * odom.x - s * odom.y;
  out.y = s * odom.x + c * odom.y;
  out.z = odom.z;
  return out;
}

void RescueMission::applyRouteFrameYaw()
{
  if (std::fabs(field_yaw_deg_) < 1e-3f)
  {
    ROS_INFO("Route frame: field_yaw_deg=0, so odom and the field frame coincide "
             "here and the route tables are used exactly as authored.");
    return;
  }

  auto rotate_route = [this](const char *route_name,
                             std::vector<geometry_msgs::Point> &route) {
    if (route.empty())
    {
      return;
    }
    const geometry_msgs::Point first = route.front();
    const geometry_msgs::Point last = route.back();
    for (std::size_t i = 0; i < route.size(); ++i)
    {
      route[i] = fieldToOdom(route[i]);
    }
    // Print both ends of every table. This is the only place where "the drawing
    // position" and "the odom position" can be compared line by line, which is
    // what makes a wrong field_yaw_deg visible in the log rather than only on
    // the aircraft.
    ROS_INFO("  %s: %zu points; first (%.2f, %.2f) -> (%.2f, %.2f), "
             "last (%.2f, %.2f) -> (%.2f, %.2f)",
             route_name, route.size(), first.x, first.y, route.front().x,
             route.front().y, last.x, last.y, route.back().x, route.back().y);
  };

  ROS_WARN("Route frame: the airframe's heading in the field frame is %.1f deg, "
           "so every field-frame route point is published rotated by %.1f deg "
           "into odom. The numbers in the launch file still describe the FIELD.",
           static_cast<double>(field_yaw_deg_),
           -static_cast<double>(field_yaw_deg_));
  rotate_route("hazard_route", hazard_route_);
  rotate_route("person_route", person_route_);
  rotate_route("return_route", return_route_);

  if (have_rescue_drop_point_)
  {
    const geometry_msgs::Point drawn = rescue_drop_point_;
    rescue_drop_point_ = fieldToOdom(rescue_drop_point_);
    rescue_drop_route_.clear();
    rescue_drop_route_.push_back(rescue_drop_point_);
    ROS_INFO("  drawn rescue drop (%.2f, %.2f) -> (%.2f, %.2f) in odom",
             drawn.x, drawn.y, rescue_drop_point_.x, rescue_drop_point_.y);
  }

  // The one number that can be compared against the map by eye, and it is
  // necessary because the measured odom yaw is 0 at boot BY CONSTRUCTION:
  // cartographer's local pose starts at identity and PX4 takes the vision yaw as
  // its reference, so reading the yaw back cannot tell a correct field_yaw_deg
  // from a wrong one. The bounding box can, and it separates all four candidates
  // on its own: the pad sits at a corner of the field rather than at its centre,
  // so no two of the four quarter turns place the safety field identically. That
  // only works because the safety field is NOT centred on the pad - a box
  // centred on the origin would be rotation-invariant and would tell you nothing.
  double min_x = 1.0e9;
  double max_x = -1.0e9;
  double min_y = 1.0e9;
  double max_y = -1.0e9;
  const float xs[2] = {safe_field_min_x_, safe_field_max_x_};
  const float ys[2] = {safe_field_min_y_, safe_field_max_y_};
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 2; ++j)
    {
      geometry_msgs::Point corner;
      corner.x = xs[i];
      corner.y = ys[j];
      corner.z = 0.0f;
      const geometry_msgs::Point rotated = fieldToOdom(corner);
      min_x = std::min(min_x, static_cast<double>(rotated.x));
      max_x = std::max(max_x, static_cast<double>(rotated.x));
      min_y = std::min(min_y, static_cast<double>(rotated.y));
      max_y = std::max(max_y, static_cast<double>(rotated.y));
    }
  }
  ROS_WARN("Route frame check, before arming: with this angle the safety field "
           "occupies x [%.2f, %.2f], y [%.2f, %.2f] in odom. The pad is a field "
           "corner, not its centre, so all four candidate angles put the "
           "rectangle somewhere different: 0 deg gives x [-0.60, 3.30] y "
           "[-3.30, 0.60], -90 gives y [-0.60, 3.30], +90 gives x [-3.30, 0.60], "
           "180 turns the box end for end. Compare this rectangle against the "
           "map; if it is a different one, this angle is wrong and the mission "
           "is flying a rotated field.",
           min_x, max_x, min_y, max_y);
}

void RescueMission::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
  const bool was_offboard = is_offboard_;
  fcu_state_ = *msg;
  is_offboard_ = msg->mode == "OFFBOARD";

  if (was_offboard && !is_offboard_ && abort_on_offboard_loss_ &&
      state_ != WAIT_FCU && state_ != LANDING && state_ != COMPLETE && state_ != ABORT)
  {
    abortMission("PX4 left OFFBOARD mode");
  }
}

void RescueMission::servoReadyCallback(const std_msgs::Bool::ConstPtr &msg)
{
  servo_ready_ = msg->data;
}

void RescueMission::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  current_position_ = msg->pose.position;
  const geometry_msgs::Quaternion &q = msg->pose.orientation;
  current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  have_pose_ = true;
  if (!have_home_)
  {
    home_position_ = current_position_;
    // Fixed mission heading, NOT the yaw measured at takeoff. The old code
    // captured current_yaw_ here, which made the entire flight's body->world
    // rotation depend on whichever noisy EKF sample happened to arrive first.
    // fix_heading_deg is a mission constant (0 deg = nose along +x in odom),
    // matching the spawn pose in px4_gazebo_slam.launch (iris is spawned with
    // -x 0 -y 0 -z 0.25 and no yaw, so its nose already starts along +x, and
    // PX4 is never asked to rotate it). Log it so a mismatch against the
    // measured yaw is visible immediately instead of silently skewing every
    // subsequent command.
    mission_heading_ = static_cast<double>(fix_heading_deg_) * kDegToRad;
    have_home_ = true;
    target_position_ = home_position_;
    ROS_INFO("Home position captured: %.3f %.3f %.3f",
             home_position_.x, home_position_.y, home_position_.z);
    ROS_INFO("Mission heading fixed at %.1f deg (odom); measured yaw at "
             "capture %.1f deg",
             static_cast<double>(fix_heading_deg_),
             current_yaw_ / kDegToRad);
  }

  try
  {
    tf::StampedTransform map_to_base;
    tf_listener_.lookupTransform("map", "base_link", ros::Time(0), map_to_base);
    double roll = 0.0;
    double pitch = 0.0;
    double map_yaw = 0.0;
    tf::Matrix3x3(map_to_base.getRotation()).getRPY(roll, pitch, map_yaw);

    std::lock_guard<std::mutex> lock(map_pose_mutex_);
    current_map_position_.x = map_to_base.getOrigin().x();
    current_map_position_.y = map_to_base.getOrigin().y();
    current_map_position_.z = map_to_base.getOrigin().z();
    if (!have_map_heading_)
    {
      // Retained as a diagnostic only. Since 2026-09-24 19:30 the whole
      // planning stack runs in odom (see publishNextGoal), so nothing steers by
      // the SLAM-measured heading any more; publishing a goal whose yaw came
      // from a drifting map frame was one more way for SLAM to aim the aircraft
      // off. If this ever reads far from mission_heading_, SLAM is wrong, not
      // the mission.
      mission_map_heading_ = map_yaw;
      have_map_heading_ = true;
    }
    have_map_pose_ = true;
  }
  catch (const tf::TransformException &error)
  {
    ROS_WARN_THROTTLE(2.0, "Mission cannot transform map -> base_link: %s",
                      error.what());
  }
}

void RescueMission::plannerVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y))
  {
    planner_velocity_ = geometry_msgs::Point();
    planner_command_received_at_ = 0.0;
    ROS_WARN_THROTTLE(1.0, "Ignoring non-finite planner command");
    return;
  }

  planner_velocity_.x = msg->linear.x;
  planner_velocity_.y = msg->linear.y;
  planner_velocity_.z = 0.0;

  // The follower already converted the plan into the odom frame, so this
  // velocity IS the world velocity and must be forwarded untouched. Rotating it
  // here as well would double-rotate.
  //
  // This is the configuration that removes the sway mechanism described in the
  // header: the old layout rotated world->body in the follower using the yaw of
  // odom->base_link and then body->world here using current_yaw_ from
  // /mavros/local_position/pose. Those two angles are both "the vehicle's yaw"
  // as seen by two different estimators, and when they disagree the two
  // rotations do not cancel - the difference survives as a rotation of the
  // velocity vector, i.e. a sideways push that the position loop cannot see,
  // because the mission pins its position setpoint to the vehicle's own
  // measured position (zero position error) and drives with velocity alone.
  // With one rotation there is nothing left to disagree. As of 2026-10-17 the
  // body-frame branch is gone, so no configuration can put a second one back.

  const float planar_speed = std::hypot(planner_velocity_.x, planner_velocity_.y);
  if (planar_speed > max_velocity_ && planar_speed > 0.0f)
  {
    const float scale = max_velocity_ / planar_speed;
    planner_velocity_.x *= scale;
    planner_velocity_.y *= scale;
  }
  planner_command_received_at_ = ros::Time::now().toSec();
  if (std::hypot(planner_velocity_.x, planner_velocity_.y) > 0.03f)
  {
    planner_motion_received_at_ = planner_command_received_at_;
  }

  // Keep the follower->PX4 path falsifiable from a log paste: `in` is what the
  // follower published, `out` is what leaves for PX4. They can differ only by
  // the speed clamp above; any rotation appearing here would be a bug.
  ROS_INFO_THROTTLE(1.0,
                    "cmd_vel: in=(%.3f, %.3f) -> out=(%.3f, %.3f) "
                    "[odom passthrough, no rotation]",
                    msg->linear.x, msg->linear.y,
                    planner_velocity_.x, planner_velocity_.y);
}

void RescueMission::gpioAckCallback(const std_msgs::String::ConstPtr &msg)
{
  const double now = ros::Time::now().toSec();
  last_gpio_ack_at_ = now;
  if (msg->data == "CAMERA_DOWN_DONE" && state_ == TAKEOFF)
  {
    camera_down_acknowledged_ = true;
    camera_down_abort_reported_ = false;
    is_offboard_ = fcu_state_.mode == "OFFBOARD";
    camera_down_ack_at_ = now;
  }
  else if (msg->data == "CAMERA_FRONT_DONE" && state_ == LANDING)
  {
    landing_camera_acknowledged_ = true;
    landing_camera_abort_reported_ = false;
    is_offboard_ = fcu_state_.mode == "OFFBOARD";
    landing_camera_ack_at_ = now;
  }
  if (msg->data == "DROP_FIRE_DONE" || msg->data == "DROP_RESCUE_DONE")
  {
    drop_acknowledged_ = true;
  }
}

void RescueMission::detectionCallback(const navigation_2d::TargetDetection::ConstPtr &msg)
{
  // Liveness is recorded before every filter below, and that ordering is the
  // whole point: the pre-arm gate asks "is the vision pipeline alive", which has
  // nothing to do with the current state or with anything having been detected.
  // Both backends publish one message per processed frame, so an idle scene
  // still produces this heartbeat.
  vision_alive_at_.store(ros::Time::now().toSec());

  if (state_ != HAZARD_HOVER && state_ != HAZARD_ALIGN &&
      state_ != RESCUE_HOVER && state_ != RESCUE_ALIGN)
  {
    return;
  }

  const std::string expected_mode = task_kind_ == TASK_HAZARD ? "hazard" : "person";
  if (msg->mode != expected_mode)
  {
    // Counted, not logged: this is the silent branch. A message that carries the
    // wrong mode used to vanish without a trace, and the task side then reported
    // "found=0" - which is exactly what a detector seeing nothing also reports.
    // The counters travel into that message instead (see the task timeouts).
    ++task_frames_seen_;
    return;
  }
  ++task_frames_seen_;
  ++task_frames_matched_;
  task_frame_at_.store(ros::Time::now().toSec());
  last_detection_ = *msg;
  task_detection_received_ = msg->found && msg->stable;
}

void RescueMission::controlCallback(const ros::TimerEvent &event)
{
  if (!have_pose_ || state_ == COMPLETE ||
      (state_ == LANDING && landing_requested_))
  {
    return;
  }

  if (landing_failed_ && !abort_land_requested_)
  {
    if (fcu_state_.mode == "OFFBOARD" && fcu_state_.armed)
    {
      geometry_msgs::Point hold_velocity;
      flight_control_.setpoint_raw_local_pos_vel(
          planner_hold_position_, hold_velocity, static_cast<float>(mission_heading_));
    }
    return;
  }

  if (state_ == WAIT_FCU)
  {
    geometry_msgs::Point hold_position = current_position_;
    geometry_msgs::Point hold_velocity;
    flight_control_.setpoint_raw_local_pos_vel(
        hold_position, hold_velocity, static_cast<float>(mission_heading_));
  }
  else if (state_ == NAV_HAZARD || state_ == NAV_PERSON || state_ == NAV_RESCUE ||
           state_ == RETURN_HOME)
  {
    // Report the two position estimates before judging the safety field. They
    // come from different places: current_position_ is PX4/mavros local
    // position (the frame the vehicle is actually flown in), current_map_
    // position_ is the SLAM map->base_link transform. If they disagree, the
    // field test below is being applied to the drifting one, and the mission
    // aborts for an excursion that never happened.
    {
      geometry_msgs::Point map_position;
      {
        std::lock_guard<std::mutex> lock(map_pose_mutex_);
        map_position = current_map_position_;
      }
      // NAV cmd is the one line that shows both position estimates together.
      // Everything except `map` is odom, and odom is what the mission flies on;
      // `map` is printed purely as a SLAM health indicator. The goal is odom now
      // too, so d is a genuine navigation error - and if `map` ever reads metres
      // away from `odom` again, the flight is unaffected, which is the whole
      // point of the 19:30 change.
      //
      // z and fly_z were added on 2026-09-25 because this line only showed x/y,
      // which made the altitude unfalsifiable from a log paste. The question it
      // now answers: current_position_ comes from /mavros/local_position/pose,
      // i.e. PX4's own estimate, NOT from the rig's ground-truth odom. So `z` is
      // what the FCU believes and fly_z is what it was told. If z holds at
      // fly_z while the Gazebo model origin in the rig's "Rig check" line sits
      // ~0.42 m higher, then the vehicle is exactly where the flight stack wants
      // it and the extra height belongs to the sensor rig model, not the
      // mission. Run `rosrun gc_2026_sim check_altitude.py` for the full chain.
      const double goal_dx = current_position_.x - target_position_odom_.x;
      const double goal_dy = current_position_.y - target_position_odom_.y;
      ROS_INFO_THROTTLE(0.5,
                        "NAV cmd: odom=(%.2f, %.2f, %.3f) map=(%.2f, %.2f) "
                        "goal=(%.2f, %.2f) d=%.3f fly_z=%.2f "
                        "vel=(%.3f, %.3f) speed=%.3f %s",
                        current_position_.x, current_position_.y,
                        current_position_.z,
                        map_position.x, map_position.y,
                        target_position_odom_.x, target_position_odom_.y,
                        std::hypot(goal_dx, goal_dy),
                        static_cast<double>(fly_height_),
                        planner_velocity_.x, planner_velocity_.y,
                        std::hypot(planner_velocity_.x, planner_velocity_.y),
                        plannerCommandFresh(ros::Time::now().toSec()) ? "planner" : "hold");
    }
    if (enforce_safe_field_ && !insideSafeField())
    {
      geometry_msgs::Point map_position;
      {
        std::lock_guard<std::mutex> lock(map_pose_mutex_);
        map_position = current_map_position_;
      }
      ROS_ERROR_THROTTLE(1.0,
                         "Vehicle left the simulation safety field: "
                         "map x=%.3f y=%.3f (odom x=%.3f y=%.3f); holding",
                         map_position.x, map_position.y,
                         current_position_.x, current_position_.y);
      abortMission("Vehicle left the simulation safety field");
      return;
    }

    geometry_msgs::Point planner_position;
    geometry_msgs::Point planner_velocity;
    const bool command_fresh = plannerCommandFresh(ros::Time::now().toSec());
    const bool command_moving = std::hypot(planner_velocity_.x, planner_velocity_.y) > 0.03f;
    if (command_fresh && command_moving)
    {
      planner_hold_active_ = false;
      planner_position = current_position_;
      planner_position.z = fly_height_;
      planner_velocity = planner_velocity_;
    }
    else
    {
      if (!planner_hold_active_)
      {
        planner_hold_position_ = current_position_;
        planner_hold_position_.z = fly_height_;
        planner_hold_active_ = true;
      }
      planner_position = planner_hold_position_;
      if (!command_fresh)
      {
        ROS_WARN_THROTTLE(1.0, "Planner command is stale; holding current position");
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "Planner is commanding zero velocity; holding current position");
      }
    }
    // Show exactly what leaves the mission node. Every drift investigation so
    // far had to guess whether the planner or the mission node was driving the
    // vehicle; this makes the answer visible. (The odom-vs-map comparison is
    // printed earlier, before the safety-field test, so it survives an abort.)
    flight_control_.setpoint_raw_local_pos_vel(
        planner_position, planner_velocity, static_cast<float>(mission_heading_));
  }
  else if (state_ == LANDING)
  {
    geometry_msgs::Point hold_velocity;
    flight_control_.setpoint_raw_local_pos_vel(
        planner_hold_position_, hold_velocity, static_cast<float>(mission_heading_));
  }
  else if (state_ == ABORT)
  {
    // 已经请求 AUTO.LAND：PX4 接管高度，这里必须闭嘴。继续发 hold setpoint
    // 会和降落抢高度，而且 OFFBOARD 的 setpoint 流会阻止模式切换稳定下来。
    if (abort_land_requested_)
    {
      return;
    }
    if (fcu_state_.mode == "OFFBOARD" && fcu_state_.armed)
    {
      geometry_msgs::Point hold_velocity;
      flight_control_.setpoint_raw_local_pos_vel(
          planner_hold_position_, hold_velocity, static_cast<float>(mission_heading_));
      // Confirm the hold command is actually going out, and show where the
      // vehicle is relative to the position it is being pinned to. If the
      // vehicle keeps moving while this prints a constant hold position, the
      // command is not reaching or not being honoured by the flight stack.
      ROS_INFO_THROTTLE(0.5,
                        "ABORT settling: odom=(%.2f, %.2f) hold=(%.2f, %.2f) "
                        "vel=(%.3f, %.3f) mode=%s armed=%d (AUTO.LAND in %.1f s)",
                        current_position_.x, current_position_.y,
                        planner_hold_position_.x, planner_hold_position_.y,
                        hold_velocity.x, hold_velocity.y,
                        fcu_state_.mode.c_str(), fcu_state_.armed ? 1 : 0,
                        std::max(0.0, static_cast<double>(abort_land_settle_) -
                                          (ros::Time::now().toSec() - state_started_at_)));
    }
    else
    {
      // Nothing is published in this state, so PX4 sees the offboard stream
      // stop. Make that visible instead of failing silently.
      ROS_WARN_THROTTLE(1.0,
                        "ABORT: not publishing a hold setpoint (mode=%s armed=%d)",
                        fcu_state_.mode.c_str(), fcu_state_.armed ? 1 : 0);
    }
  }
  else if (state_ == TAKEOFF)
  {
    // 起飞用**纯位置**设定点 + z 斜坡，位置环全权交给 PX4。
    //
    // 以前起飞和悬停/对齐共用下面那一支：velocity = 0.8 * (目标 - 实测)，
    // position = 实测 + 0.2 * velocity。setpoint_raw_local_pos_vel 的 type_mask
    // 只忽略加速度/FORCE/yaw_rate（flight_control.cpp:124），所以位置和速度
    // **两个字段同时有效**，PX4 拿到的是"速度前馈 + 位置 P"两条并行的位置环，
    // 而且位置设定点是从实测位姿算出来的，中间只隔着 mavros + EKF 的二三十
    // 毫秒延迟。0.2 s 的提前量恰好把这延迟抵消掉，净阻尼 ≈ 0：离地瞬间的任何
    // 扰动都不衰减，只以速度环的带宽来回荡几下，而且稳态会偏到
    // (速度跟踪偏差 / 0.95) 处 —— 偏 10 cm 以内 atPosition 的 0.12 m 窗口
    // 根本看不出来。
    //
    // 2026-10-18 确认仿真同样晃动，而 SITL 里没有任何节点发
    // /mavros/vision_pose/pose，位置估计就是 PX4 自己的 EKF：激励不是 SLAM
    // 跳变，是这个"设定点跟着实测走"的结构本身在放大它。起飞的 XY 目标是常数
    // （home），不需要那个软启动，所以这里把它整段换掉；悬停/对齐那一支目标
    // 会跳，软启动对它们仍然有意义，保留原样。
    const double dt = std::min(0.10, std::max(0.0, (event.current_real - event.last_real).toSec()));
    if (takeoff_cmd_z_ < static_cast<double>(fly_height_))
    {
      takeoff_cmd_z_ = std::min(static_cast<double>(fly_height_),
                                takeoff_cmd_z_ + static_cast<double>(max_velocity_) * dt);
    }
    geometry_msgs::Point position = home_position_;
    position.z = takeoff_cmd_z_;
    flight_control_.setpoint_raw_local_pos(position,
                                           static_cast<float>(mission_heading_));
    ROS_INFO_THROTTLE(0.5,
                      "TAKEOFF setpoint: pos=(%.2f, %.2f, %.2f) climb=%.2f m/s "
                      "odom=(%.2f, %.2f, %.2f) d=%.3f",
                      position.x, position.y, position.z,
                      static_cast<double>(max_velocity_),
                      current_position_.x, current_position_.y, current_position_.z,
                      std::hypot(current_position_.x - home_position_.x,
                                 current_position_.y - home_position_.y));
  }
  else
  {
    geometry_msgs::Point velocity;
    velocity.x = getVelocity(target_position_.x - current_position_.x, xy_gain_);
    velocity.y = getVelocity(target_position_.y - current_position_.y, xy_gain_);
    velocity.z = getVelocity(target_position_.z - current_position_.z, z_gain_);

    geometry_msgs::Point position;
    position.x = current_position_.x + velocity.x * 0.2f;
    position.y = current_position_.y + velocity.y * 0.2f;
    position.z = current_position_.z + velocity.z * 0.2f;
    flight_control_.setpoint_raw_local_pos_vel(
        position, velocity, static_cast<float>(mission_heading_));
  }

  if (preflight_setpoints_ < 200)
  {
    ++preflight_setpoints_;
  }
}

void RescueMission::missionCallback(const ros::TimerEvent &event)
{
  (void)event;
  // A stage-2 run without a usable drawn position does not fly. Dropping the
  // rescue material in the wrong place scores nothing and cannot be undone, so
  // the only safe response to a missing parameter is to stay on the ground with
  // the reason in the log, where whoever launched it will see it.
  if (rescue_drop_enabled_ && !have_rescue_drop_point_)
  {
    ROS_ERROR_THROTTLE(2.0,
                       "Stage %d is configured without a usable rescue drop "
                       "waypoint; not starting. Pass rescue_drop_x and "
                       "rescue_drop_y inside the field (x %.2f..%.2f, y %.2f..%.2f).",
                       stage_, safe_field_min_x_, safe_field_max_x_,
                       safe_field_min_y_, safe_field_max_y_);
    return;
  }
  if (!have_pose_ || !have_home_)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(map_pose_mutex_);
    if (!have_map_pose_)
    {
      ROS_WARN_THROTTLE(2.0, "Mission waiting for map -> base_link transform");
      return;
    }
  }

  const double now = ros::Time::now().toSec();
  switch (state_)
  {
  case WAIT_FCU:
    if (!fcu_state_.connected || (require_servo_ready_ && !servo_ready_))
    {
      return;
    }
    if (!auto_start_)
    {
      if (fcu_state_.armed && is_offboard_)
      {
        // Manual start: the pilot owns the decision, so this does not block.
        // But "the detector was not up when we armed" has to be in the log
        // beside the arm event, otherwise the next post-mortem repeats the
        // 2026-10-07 guesswork. Deliberately does not go through
        // visionReadyForTakeoff(): that one owns the auto-start wait state and
        // would print "refusing to take off" while the vehicle is already
        // lifting.
        if (require_vision_ready_)
        {
          const double alive_at = vision_alive_at_.load();
          if (!(alive_at > 0.0 && now - alive_at <= 2.0))
          {
            if (alive_at > 0.0)
            {
              ROS_WARN("Arming with no live vision heartbeat: the last detection "
                       "message was %.1f s ago (manual start, so the gate does "
                       "not block).", now - alive_at);
            }
            else
            {
              ROS_WARN("Arming before the vision backend produced any message at "
                       "all (manual start, so the gate does not block). The task "
                       "phase will fall back to the authored waypoint if nothing "
                       "is detected.");
            }
          }
        }
        enterState(TAKEOFF);
      }
      return;
    }
    // Before this gate existed the vehicle armed on "FCU connected + servos +
    // 100 setpoints" and took off ~2 s after mavros appeared, i.e. ~25 s before
    // the Jetson's TensorRT engine had finished loading. See the note on
    // require_vision_ready_ in the header.
    if (!visionReadyForTakeoff(now))
    {
      return;
    }
    if (preflight_setpoints_ < 100)
    {
      return;
    }
    if (!is_offboard_ && now - last_mode_request_at_ > 1.0)
    {
      flight_control_.set_mode("OFFBOARD");
      last_mode_request_at_ = now;
      return;
    }
    if (is_offboard_ && !fcu_state_.armed && now - last_arm_request_at_ > 1.0)
    {
      flight_control_.set_arm(true);
      last_arm_request_at_ = now;
      return;
    }
    if (is_offboard_ && fcu_state_.armed)
    {
      enterState(TAKEOFF);
    }
    break;

  case TAKEOFF:
    target_position_ = home_position_;
    target_position_.z = fly_height_;
    // One-shot diagnostic: show the odom frame vs the SLAM map frame side by
    // side. cartographer defines the map origin from its first scan, so the map
    // origin can sit far from the PX4 odom origin. The safety field and the
    // route goals are absolute coordinates and the route frame is the field
    // frame rotated into odom by field_yaw_deg, so this offset is the SLAM
    // health indicator rather than something the flight depends on.
    if (!origin_offset_logged_)
    {
      geometry_msgs::Point map_position = current_map_position_;
      {
        std::lock_guard<std::mutex> lock(map_pose_mutex_);
        map_position = current_map_position_;
      }
      ROS_INFO("Frame check at takeoff: odom=(%.3f, %.3f, %.3f) "
               "map=(%.3f, %.3f, %.3f) offset=(%.3f, %.3f) "
               "field_yaw_deg=%.1f (routes rotated by %.1f into odom)",
               home_position_.x, home_position_.y, home_position_.z,
               map_position.x, map_position.y, map_position.z,
               map_position.x - home_position_.x,
               map_position.y - home_position_.y,
               static_cast<double>(field_yaw_deg_),
               -static_cast<double>(field_yaw_deg_));
      origin_offset_logged_ = true;
    }
    if (atPosition(target_position_, position_tolerance_, altitude_tolerance_) &&
        now - state_started_at_ > waypoint_dwell_)
    {
      if (camera_down_acknowledged_ && fcu_state_.mode != "OFFBOARD")
      {
        ROS_FATAL_THROTTLE(1.0,
                           "PX4 left OFFBOARD during takeoff; allowing autopilot failsafe");
      }
      if (!camera_down_requested_)
      {
        requestCameraDown();
      }
      if (camera_down_acknowledged_ && fcu_state_.mode == "OFFBOARD" &&
          now - camera_down_ack_at_ >= camera_down_post_move_settle_)
      {
        startRoute(hazard_route_, NAV_HAZARD);
      }
      else if (fcu_state_.mode == "OFFBOARD" &&
               now - camera_down_started_at_ > camera_command_timeout_ &&
               !camera_down_abort_reported_)
      {
        camera_down_abort_reported_ = true;
        abortMission("Camera did not reach down-facing position after takeoff");
      }
      else if (!camera_down_acknowledged_ &&
               now - camera_down_last_sent_at_ >= camera_command_retry_interval_)
      {
        requestCameraDown();
      }
    }
    else
    {
      // The one state with no timeout, and its exit test is a position
      // predicate - so a vehicle that cannot reach home + fly_height sits here
      // forever with /trigger_plan false, and the only visible symptom is the
      // follower's own "planning disabled, commanding zero velocity" every 5 s,
      // which sends the reader to the planner instead of to the takeoff gate.
      // Print the residual and which gate is still open. Grace period first so
      // that a normal climb stays quiet.
      const double state_age = now - state_started_at_;
      if (state_age > 2.0)
      {
        ROS_WARN_THROTTLE(2.0,
                          "TAKEOFF waiting %.1f s: target=(%.2f, %.2f, %.2f) "
                          "odom=(%.2f, %.2f, %.2f) residual=(%.2f, %.2f, %.2f) "
                          "tol=(%.2f, %.2f) dwell=%s "
                          "camera_down(requested=%d ack=%d) mode=%s",
                          state_age,
                          target_position_.x, target_position_.y, target_position_.z,
                          current_position_.x, current_position_.y, current_position_.z,
                          target_position_.x - current_position_.x,
                          target_position_.y - current_position_.y,
                          target_position_.z - current_position_.z,
                          static_cast<double>(position_tolerance_),
                          static_cast<double>(altitude_tolerance_),
                          (state_age > static_cast<double>(waypoint_dwell_)) ? "ok" : "pending",
                          camera_down_requested_ ? 1 : 0,
                          camera_down_acknowledged_ ? 1 : 0,
                          fcu_state_.mode.c_str());
      }
    }
    break;

  case NAV_HAZARD:
  case NAV_PERSON:
  case NAV_RESCUE:
  case RETURN_HOME:
    if (routeComplete())
    {
      finishRoute();
      break;
    }

    // Route navigation is a horizontal problem: this node pins z to fly_height
    // every cycle and move_base only commands x/y. Use the generous route
    // altitude window so a small height offset cannot be mistaken for an
    // unreached waypoint (which previously triggered a stall/re-send loop).
    if (atRoutePosition(target_position_, position_tolerance_, route_altitude_tolerance_))
    {
      if (now - route_goal_started_at_ > waypoint_dwell_)
      {
        ++route_index_;
        if (routeComplete())
        {
          finishRoute();
        }
        else
        {
          navigation_retries_ = 0;
          publishNextGoal();
        }
      }
    }
    else
    {
      // A leg on which the planner has not produced a single MOVING command
      // since the goal was published is dead, and it is dead for a reason that
      // re-sending the goal cannot change. The case that burned 2026-10-17: EGO
      // refused every plan because the vehicle (or the waypoint) sat inside the
      // inflated band, so the follower published nothing but zeros, and this
      // node sat through the generic 10 s stall timer twice while the aircraft
      // hovered ~0.15 m short of hazard_route[1]. Report the specific cause
      // first and abort, rather than retrying into the same wall.
      //
      // planner_motion_received_at_ is reset to 0 by publishNextGoal, so it is
      // exactly "has this leg ever moved". A transient local-planning hiccup
      // cannot reach this: the follower runs at 20 Hz, so the window is over a
      // hundred missed cycles.
      if (planner_motion_received_at_ <= 0.0 &&
          now - route_goal_started_at_ > static_cast<double>(route_no_motion_timeout_))
      {
        const double to_goal = std::hypot(current_position_.x - target_position_.x,
                                          current_position_.y - target_position_.y);
        ROS_ERROR("Waypoint %.2f, %.2f never moved the vehicle: %.1f s since the "
                  "goal was published, %.3f m still to go (position_tolerance "
                  "%.2f), and no moving /cmd_vel ever arrived. The planner is "
                  "refusing this goal - read EGO's 'drone is in obstacle' / "
                  "window_end_inflated / goal_inflated lines before the next leg.",
                  target_position_.x, target_position_.y,
                  now - route_goal_started_at_, to_goal,
                  static_cast<double>(position_tolerance_));
        abortMission("Planner never started moving toward the waypoint");
        break;
      }

      const bool planner_has_stopped = planner_motion_received_at_ <= 0.0 ||
                                       now - planner_motion_received_at_ > planner_failure_abort_timeout_;
      const bool planner_stalled = now - route_goal_started_at_ > planner_failure_abort_timeout_ &&
                                   (!plannerCommandFresh(now) || planner_has_stopped);
      if (planner_stalled && navigation_retries_ < max_navigation_retries_)
      {
        ++navigation_retries_;
        // Print what will be gained or lost by the retry. If the vehicle is
        // metres away the leg died early; if it is centimetres away the retry
        // cannot help at all, because the planner is refusing this endpoint -
        // 2026-10-17 sat 0.097 m out through two identical retries.
        const double to_goal = std::hypot(current_position_.x - target_position_.x,
                                          current_position_.y - target_position_.y);
        ROS_WARN("Planner stalled %.1f m from waypoint (%.2f, %.2f); resending it, "
                 "retry %d/%d. If this repeats at the same distance, the goal or "
                 "the vehicle is inside EGO's inflated map and the retry is "
                 "pointless - check EGO's goal_inflated / 'drone is in obstacle'.",
                 to_goal, target_position_.x, target_position_.y,
                 navigation_retries_, max_navigation_retries_);
        publishNextGoal();
      }
      else if (planner_stalled)
      {
        abortMission("Planner command timeout");
      }
      else if (now - route_goal_started_at_ > navigation_timeout_)
      {
        if (navigation_retries_ < max_navigation_retries_)
        {
          ++navigation_retries_;
          ROS_WARN("Navigation timeout, retry %d/%d", navigation_retries_, max_navigation_retries_);
          publishNextGoal();
        }
        else
        {
          abortMission("Navigation timeout");
        }
      }
    }
    break;

  case HAZARD_HOVER:
  case HAZARD_ALIGN:
    processTask(TASK_HAZARD);
    break;
  case HAZARD_DROP:
    if (now - state_started_at_ > drop_settle_time_ &&
        (drop_acknowledged_ || now - state_started_at_ > drop_settle_time_ + 1.0))
    {
      target_position_.z = fly_height_;
      if (atPosition(target_position_, position_tolerance_, altitude_tolerance_))
      {
        // 人员巡游一开始就把视觉模式切过去，别等 RESCUE_HOVER 的 beginTask。
        // 以前整段 tour 里检测节点还停在 hazard 模式，人员卡全是 person zone，
        // 于是 MissionVision 每 2 s 刷一条 "The detector reported 2 'person'-zone
        // box(es) and nothing in the 'hazard' zone this mission is waiting for"
        // （2026-10-18 就是这样刷的）—— 而这段时间任务根本不读检测。那条 WARN
        // 是给"真在等目标却等错了 zone"用的，不该在巡航时响；让它响，它就会像
        // 今天这样被当成噪声忽略。模式应该描述当前阶段，不是到了悬停才追认。
        setVisionMode(MissionVision::PERSON);
        // Straight into the tour of every person task area. Whether anything
        // follows that tour is decided in finishRoute, not here.
        startRoute(person_route_, NAV_PERSON);
      }
    }
    break;

  case RESCUE_HOVER:
  case RESCUE_ALIGN:
    processTask(TASK_RESCUE);
    break;
  case RESCUE_DROP:
    if (now - state_started_at_ > drop_settle_time_ &&
        (drop_acknowledged_ || now - state_started_at_ > drop_settle_time_ + 1.0))
    {
      target_position_.z = fly_height_;
      if (atPosition(target_position_, position_tolerance_, altitude_tolerance_))
      {
        startRoute(return_route_, RETURN_HOME);
      }
    }
    break;

  case LANDING:
    if (!landing_camera_acknowledged_)
    {
      if (!fcu_state_.connected)
      {
        break;
      }
      const bool offboard_lost = fcu_state_.mode != "OFFBOARD";
      const bool timed_out = fcu_state_.mode == "OFFBOARD" &&
                             camera_front_started_at_ > 0.0 &&
                             now - camera_front_started_at_ > camera_command_timeout_;
      if ((offboard_lost || timed_out) && !landing_camera_abort_reported_)
      {
        landing_camera_abort_reported_ = true;
        if (offboard_lost)
        {
          landing_failed_ = true;
          ROS_FATAL("PX4 left OFFBOARD before camera-front confirmation; waiting for autopilot failsafe");
        }
        else
        {
          landing_failed_ = true;
          ROS_FATAL("Camera did not confirm front-facing position before landing");
        }
        abortMission("Camera-front positioning failed before landing");
      }
      if (!landing_failed_ &&
          now - camera_front_last_sent_at_ >= camera_command_retry_interval_)
      {
        requestCameraFront();
      }
      break;
    }
    if (now - landing_camera_ack_at_ < landing_camera_post_move_settle_)
    {
      break;
    }
    if (!landing_requested_)
    {
      flight_control_.set_mode("AUTO.LAND");
      landing_requested_ = true;
      state_started_at_ = now;
      ROS_WARN("Camera front-facing and settled; AUTO.LAND requested");
    }
    if (!fcu_state_.armed)
    {
      enterState(COMPLETE);
    }
    else if (now - state_started_at_ > landing_timeout_)
    {
      ROS_ERROR_THROTTLE(2.0, "Landing has not disarmed the vehicle yet");
    }
    break;

  case COMPLETE:
    setVisionMode(MissionVision::IDLE);
    ROS_WARN("Mission complete: vehicle disarmed");
    break;
  case ABORT:
  {
    // 中止之后必须落地。2026-10-17 那次：mission 在人员区以 "Planner never
    // started moving toward the waypoint" 中止，然后按旧设计 hold 在原地
    // （离家 3 m > abort_land_home_radius_），日志里再没有任何要降落的迹象，
    // 直到 EKF 的高度估计自己崩掉。对一个 4 m 的场地来说，"远"不是拒绝降落
    // 的理由 —— 不降落才是丢失飞机的理由。
    //
    // settle 的意义：abort 往往发生在规划器刚拒绝目标、速度环还没收敛的时候，
    // 立刻切 AUTO.LAND 会把残余速度带进降落。原地保持 abort_land_settle_ 秒。
    if (!fcu_state_.armed)
    {
      ROS_WARN("Abort: vehicle disarmed (mode=%s); mission complete",
               fcu_state_.mode.c_str());
      enterState(COMPLETE);
      break;
    }
    if (!abort_land_requested_)
    {
      if (now - state_started_at_ < static_cast<double>(abort_land_settle_))
      {
        break;
      }
      flight_control_.set_mode("AUTO.LAND");
      abort_land_requested_ = true;
      abort_land_requested_at_ = now;
      ROS_WARN("Abort: AUTO.LAND requested at odom (%.2f, %.2f, %.3f), %.2f m from "
               "home. Landing here may put the aircraft down next to an obstacle - "
               "that is the trade for not leaving it hovering until the battery ends.",
               current_position_.x, current_position_.y, current_position_.z,
               std::hypot(current_position_.x - home_position_.x,
                          current_position_.y - home_position_.y));
      break;
    }
    if (now - abort_land_requested_at_ > static_cast<double>(abort_land_retry_interval_))
    {
      abort_land_requested_at_ = now;
      flight_control_.set_mode("AUTO.LAND");
      ROS_ERROR_THROTTLE(2.0,
                         "Abort: still armed %.1f s after the AUTO.LAND request "
                         "(mode=%s); resending. If the mode never changes the switch "
                         "is being rejected - take it down manually.",
                         now - state_started_at_, fcu_state_.mode.c_str());
    }
    break;
  }
  }
}

void RescueMission::startRoute(const std::vector<geometry_msgs::Point> &route, State state)
{
  active_route_ = &route;
  route_index_ = 0;
  navigation_retries_ = 0;
  state_ = state;
  state_started_at_ = ros::Time::now().toSec();
  // A route is about to be flown, so the planner has something to do. This has
  // to happen before publishNextGoal(), otherwise the goal could reach a planner
  // that is still switched off. startRoute sets state_ itself rather than going
  // through enterState, so it cannot rely on enterState to do this.
  setPlanningEnabled(true);
  planner_velocity_ = geometry_msgs::Point();
  planner_hold_position_ = geometry_msgs::Point();
  planner_command_received_at_ = 0.0;
  planner_motion_received_at_ = 0.0;
  planner_hold_active_ = false;
  target_position_.z = fly_height_;
  ROS_INFO("Mission state: %s", stateName(state_));

  if (route.empty())
  {
    ROS_WARN("Route is empty for state %s", stateName(state_));
    return;
  }
  publishNextGoal();
}

void RescueMission::publishNextGoal()
{
  if (active_route_ == nullptr || routeComplete())
  {
    return;
  }

  target_position_ = (*active_route_)[route_index_];
  if (target_position_.z <= 0.05f)
  {
    target_position_.z = fly_height_;
  }

  // The goal is published in ODOM, not map. This single line is the fix for
  // "the drone keeps hitting the wall" and it is deliberate, so the reasoning
  // stays here.
  //
  // It used to be frame_id = "map", with a separate map->odom conversion for
  // the arrival test. Either way, map->odom - a transform owned by cartographer,
  // which both drifts and can stop updating altogether - sat on the critical
  // path of every waypoint. Two independently reproduced failures on
  // 2026-09-24:
  //
  //   19:19 run. SLAM drifted the map frame ~2.2-3.0 m north (false loop
  //   closures), so a waypoint authored inside the field arrived in odom
  //   BEYOND the south wall at y = -3.375. The controller dutifully drove south
  //   and oscillated against the wall for 20 s. Nothing was mis-tuned; the goal
  //   it was handed was unreachable.
  //
  //   19:26 run (TEB). cartographer's front end stalled - its TF bridge could
  //   not satisfy an odom->base_link lookup at a stamp its 10 s buffer had
  //   already pruned ("Lookup would require extrapolation 5.456 s into the past.
  //   Requested time 40.900000 but the earliest data is at time 46.356000"), so
  //   map->odom froze at pose stamp 50.888 s while sim time ran to 56.4 s.
  //   Every map-frame consumer died with it: "Transform timeout for
  //   global_costmap. Current time: 56.4000, pose stamp: 50.8880", "Could not
  //   transform the global plan to the frame of the controller", "Could not get
  //   local plan", "DWA planner failed to produce path", and NAV cmd reporting
  //   map=(3.46, 0.84) while odom stayed at (-0.37, -0.49). The vehicle kept
  //   executing its last command until it hit a wall.
  //
  // odom is the correct frame here, and the baseline's own code says so even
  // though it spells the frame "map".
  //
  // Read navigation_2d/src/shoot_with_move_base.cpp in catkin_ws_bak: it
  // publishes the goal as frame_id = "map" (line 543) but its arrival test
  // compares the same fixed numbers against current_position, which is filled
  // straight from /mavros/local_position/pose - i.e. odom - at a 0.03 m
  // tolerance, with no TF conversion anywhere (line 162). Two frames, one set
  // of numbers, no transform. That can only be correct when map and odom are
  // the same thing to within 3 cm on the day, which is a property of last
  // year's venue and of a real RPLidar S2, not a property of the parameter
  // file. It is a coincidence the baseline was allowed to rely on; copying the
  // frame NAME would be copying the coincidence, not copying a parameter.
  //
  // In this simulation the coincidence is removed by construction. The
  // obstacles come in mirrored pairs (cylinders at x 0.70/2.10, blocks at
  // x 0.70/2.10), which is exactly the geometry that produces aliased loop
  // closures, and the observed failure was a rigid ~2.24 m translation of the
  // whole map with zero rotation. Meanwhile odom is exact: px4_sensor_rig_node
  // derives odom->base_link from the Gazebo ground truth, so it cannot drift,
  // and its origin is the field origin the routes are authored in - see
  // "Frame check at takeoff: odom=(-0.001, 0.000, -0.019) map=(0.009, 0.008,
  // 0.000)". The route numbers (-0.25/-1.80/1.50 ...) are field-frame numbers,
  // and in this sim the field frame IS odom.
  //
  // Four consumers now read the same frame and therefore cannot disagree:
  // this goal, the arrival test (atRoutePosition, via current_position_), the
  // safety field (insideSafeField) and the velocity integration in
  // controlCallback. Nothing between the route table and the motors needs SLAM
  // any more, so a SLAM stall or a SLAM jump can no longer move a waypoint.
  //
  // The two costmaps deliberately stay in map (the baseline's choice, see
  // local_costmap_params.yaml / global_costmap_sitl.yaml) and that is safe:
  // they receive this goal through TF, so a rigid SLAM offset moves the goal
  // and the walls by the same amount and cancels out. What does not cancel is
  // handing move_base the literal route numbers stamped "map" - that places
  // the waypoint at T_odom_map * goal, i.e. off by the SLAM error.
  //
  // Consequence: the arrival test needs no conversion at all - target_position_
  // and target_position_odom_ are now the same point. target_odom_valid_ is
  // kept only so the NAV cmd log keeps its last field.
  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "odom";
  goal.pose.position = target_position_;
  // mission_heading_, not mission_map_heading_: the goal now lives in odom, so
  // the heading must be the one the flight controller is actually holding.
  goal.pose.orientation.z = std::sin(mission_heading_ * 0.5);
  goal.pose.orientation.w = std::cos(mission_heading_ * 0.5);
  goal_pub_.publish(goal);

  target_position_odom_ = target_position_;
  target_odom_valid_ = true;
  ROS_INFO("Route goal in odom frame: %.2f %.2f (goal frame: odom, no conversion)",
           target_position_odom_.x, target_position_odom_.y);
  route_goal_started_at_ = ros::Time::now().toSec();
  planner_velocity_ = geometry_msgs::Point();
  planner_hold_position_ = geometry_msgs::Point();
  planner_command_received_at_ = 0.0;
  planner_motion_received_at_ = 0.0;
  planner_hold_active_ = false;
  ROS_INFO("Route goal %zu/%zu: %.2f %.2f %.2f", route_index_ + 1,
           active_route_->size(), target_position_.x, target_position_.y,
           target_position_.z);
}

bool RescueMission::routeComplete() const
{
  return active_route_ == nullptr || route_index_ >= active_route_->size();
}

void RescueMission::finishRoute()
{
  switch (state_)
  {
  case NAV_HAZARD:
    enterState(HAZARD_HOVER);
    break;
  case NAV_PERSON:
    // The last of the three person task areas has been flown over. This line is
    // the entire difference between the two competitions: round 2 has one more
    // payload and therefore one more leg (to the drawn position) before it goes
    // home, round 1 goes home from here.
    //
    // Note what is NOT here any more: the tour used to end in a hover/align/drop
    // at whichever person card the route happened to stop on, and the rescue
    // material then had to be dropped there. That conflated "cover every person
    // task area" with "drop at the drawn position", which are two different
    // requirements, and it is why the tour only ever reached one card.
    if (rescue_drop_enabled_)
    {
      ROS_INFO("Person task areas complete; flying to the drawn rescue drop "
               "position (%.2f, %.2f)", rescue_drop_point_.x, rescue_drop_point_.y);
      startRoute(rescue_drop_route_, NAV_RESCUE);
    }
    else
    {
      ROS_INFO("Person task areas complete; returning home (stage %d has no "
               "rescue-material drop)", stage_);
      enterState(RETURN_HOME);
    }
    break;
  case NAV_RESCUE:
    enterState(RESCUE_HOVER);
    break;
  case RETURN_HOME:
    enterState(LANDING);
    break;
  default:
    break;
  }
}

void RescueMission::enterState(State state)
{
  state_ = state;
  state_started_at_ = ros::Time::now().toSec();
  ROS_INFO("Mission state: %s", stateName(state_));

  // Leaving a route phase must release move_base, otherwise its last waypoint
  // stays active and keeps publishing velocity while we hover/drop/land.
  //
  // The same predicate drives /trigger_plan, because "leaving a route phase" and
  // "the planner has nothing left to do" are the same condition. Without this the
  // ego backend kept replanning through hover, drop, landing and completion, and
  // kept publishing /visual_local_trajectory at 5 Hz for as long as the node
  // lived - the reported "it is still planning after it landed".
  const bool planner_active =
      (state_ == NAV_HAZARD || state_ == NAV_PERSON || state_ == NAV_RESCUE ||
       state_ == RETURN_HOME);
  setPlanningEnabled(planner_active);
  if (!planner_active)
  {
    cancelPlannerGoal();
  }

  switch (state_)
  {
  case TAKEOFF:
    // 斜坡从"现在在哪"起步，不是从 0 起步，所以设定点不会在进状态的第一帧
    // 上跳。取 min 是为了从高于 fly_height 的地方手动起任务时也能往下收。
    takeoff_cmd_z_ = std::min(static_cast<double>(current_position_.z),
                              static_cast<double>(fly_height_));
    break;
  case HAZARD_HOVER:
    beginTask(TASK_HAZARD);
    break;
  case RESCUE_HOVER:
    beginTask(TASK_RESCUE);
    break;
  case HAZARD_DROP:
    beginDrop(TASK_HAZARD);
    break;
  case RESCUE_DROP:
    beginDrop(TASK_RESCUE);
    break;
  case RETURN_HOME:
    setVisionMode(MissionVision::IDLE);
    startRoute(return_route_, RETURN_HOME);
    break;
  case LANDING:
    landing_camera_acknowledged_ = false;
    landing_failed_ = false;
    landing_camera_abort_reported_ = false;
    camera_front_started_at_ = 0.0;
    camera_front_last_sent_at_ = 0.0;
    landing_camera_ack_at_ = 0.0;
    landing_requested_ = false;
    planner_hold_position_ = current_position_;
    requestCameraFront();
    beginLanding();
    break;
  case COMPLETE:
    setVisionMode(MissionVision::IDLE);
    ROS_WARN("Mission complete: vehicle disarmed");
    break;
  default:
    break;
  }
}

void RescueMission::beginTask(TaskKind kind)
{
  task_kind_ = kind;
  task_detection_received_ = false;
  task_alignment_started_ = false;
  alignment_started_at_ = 0.0;
  align_entered_at_ = 0.0;
  last_detection_ = navigation_2d::TargetDetection();
  task_frames_seen_ = 0;
  task_frames_matched_ = 0;
  task_frame_at_ = 0.0;
  detection_started_at_ = ros::Time::now().toSec();
  target_position_ = current_position_;
  target_position_.z = task_height_;

  // The authored drop waypoint - the one thing the fallback mode is allowed to
  // fall back TO - in odom.
  //
  // Hazard: the last hazard_route waypoint. That is deliberate rather than a
  // second copy of the number, because it is exactly the point that was just
  // published as the final goal, so "the route flew here" and "the fallback
  // drops here" cannot drift apart. hazard_target_index and that last waypoint
  // are a pair: pointing the camera at one card while the route stops over the
  // other is a configuration error the fallback would then hide, which is why
  // tools/check_config.py checks the pair against the world file.
  // Rescue: the drawn position from rescue_drop_x/rescue_drop_y.
  task_reference_ = (kind == TASK_HAZARD && !hazard_route_.empty())
                        ? hazard_route_.back()
                        : rescue_drop_point_;

  setVisionMode(kind == TASK_HAZARD ? MissionVision::HAZARD : MissionVision::PERSON);
  ROS_INFO("%s task: hover at %.2f m, drop reference waypoint (%.2f, %.2f), "
           "align mode %s",
           taskName(kind), static_cast<double>(task_height_),
           task_reference_.x, task_reference_.y,
           dropAlignModeName(drop_align_mode_));
  if (kind == TASK_HAZARD && hazard_route_.empty())
  {
    ROS_WARN("hazard_route is empty, so the hazard drop has no reference "
             "waypoint; the waypoint fallback is unavailable for this drop");
  }
}

void RescueMission::processTask(TaskKind kind)
{
  const double now = ros::Time::now().toSec();
  if (task_kind_ != kind)
  {
    return;
  }

  const State hover_state = hoverState(kind);
  const State align_state = alignState(kind);
  const State drop_state = dropState(kind);

  if (state_ == hover_state)
  {
    if (now - state_started_at_ < camera_settle_time_ ||
        std::fabs(current_position_.z - task_height_) > altitude_tolerance_)
    {
      return;
    }

    // Waypoint mode: the camera is not consulted at all. The route has already
    // stopped the vehicle on the authored waypoint, so all that is left is to
    // confirm it really is there. ALIGN is reused as the settle step: in this
    // mode target_position_ is the position the vehicle is already holding, so it
    // passes immediately and only the alignment dwell separates this from the
    // servo command.
    if (drop_align_mode_ == DROP_ALIGN_WAYPOINT)
    {
      if (!atDropWaypoint())
      {
        ROS_WARN_THROTTLE(1.0,
                          "%s waypoint drop: %.3f m from the reference waypoint "
                          "(%.2f, %.2f), tolerance %.2f m; holding instead of dropping",
                          taskName(kind), distanceToDropWaypoint(),
                          task_reference_.x, task_reference_.y,
                          static_cast<double>(drop_waypoint_tolerance_));
        // Waiting here is bounded on purpose. Without a camera there is nothing
        // that will ever change on its own, so an unbounded wait would be a
        // silent hover until the battery ends the flight.
        if (now - state_started_at_ > task_timeout_)
        {
          abortMission(kind == TASK_HAZARD
                           ? "Hazard waypoint drop: never reached the reference waypoint"
                           : "Rescue waypoint drop: never reached the reference waypoint");
        }
        return;
      }
      ROS_INFO("%s drop: waypoint mode, vision not used; dropping on (%.2f, %.2f) "
               "from %.3f m",
               taskName(kind), task_reference_.x, task_reference_.y,
               distanceToDropWaypoint());
      alignment_started_at_ = now;
      align_entered_at_ = now;
      enterState(align_state);
      return;
    }

    if (!task_detection_received_)
    {
      // Say it out loud the moment the channel goes quiet, not five seconds later
      // in a message that cannot tell the two apart. Everything downstream of this
      // point - the fallback, the alignment, the abort - is only meaningful if the
      // vision node is still talking; if it is not, the interesting failure is
      // upstream and this line is what points there.
      const double frame_age = task_frame_at_.load() > 0.0
                                   ? now - task_frame_at_.load()
                                   : now - detection_started_at_;
      if (frame_age > static_cast<double>(vision_silence_timeout_))
      {
        ROS_WARN_THROTTLE(1.0,
                          "%s: no /mission/vision/detection frame for %.1fs (%s). "
                          "Boxes still drawn on /ssd_detection/image/compressed do "
                          "not contradict this: that image is produced inside the "
                          "detector node, upstream of every filter here.",
                          taskName(kind), frame_age, visionWaitReport().c_str());
      }

      // Nothing stable yet. In fallback mode the camera gets drop_vision_timeout_
      // and then the authored waypoint takes over. A detection failure must not
      // be able to move the drop to a place nobody chose - which is why the
      // fallback goes TO the waypoint rather than just dropping where the vehicle
      // happens to be.
      if (drop_align_mode_ == DROP_ALIGN_VISION_FALLBACK &&
          now - detection_started_at_ > static_cast<double>(drop_vision_timeout_))
      {
        if (atDropWaypoint())
        {
          ROS_WARN("%s: no stable detection after %.1fs (last: found=%d stable=%d; %s); "
                   "falling back to the authored waypoint (%.2f, %.2f), %.3f m away. "
                   "The payload lands where the position estimate says, not where "
                   "the target is - check the video before trusting this drop.",
                   taskName(kind), static_cast<double>(drop_vision_timeout_),
                   last_detection_.found ? 1 : 0, last_detection_.stable ? 1 : 0,
                   visionWaitReport().c_str(),
                   task_reference_.x, task_reference_.y, distanceToDropWaypoint());
          // Enter ALIGN holding the CURRENT point, not the waypoint. The 0.20 m
          // above is the accuracy this fallback promises; ALIGN's 0.08 m gate
          // belongs to vision corrections. Re-aiming ALIGN at the waypoint would
          // turn "already close enough" into a second, tighter convergence that
          // can time out - and an align timeout aborts, so that would trade a
          // certain drop for a possible one.
          target_position_ = current_position_;
          target_position_.z = task_height_;
          alignment_started_at_ = now;
          align_entered_at_ = now;
          enterState(align_state);
          return;
        }
        // Not there yet. Fly the remaining distance onto the authored waypoint
        // instead of holding still until task_timeout_ aborts the mission.
        //
        // 2026-10-18: this branch used to only warn ("dropping here would be
        // dropping blind") and then wait for the abort. So "no stable detection"
        // still ended in no drop whenever the route had stopped just outside
        // drop_waypoint_tolerance_ - which is the one case this fallback exists
        // for. The waypoint is not a blind drop: it is the last goal the route
        // published, and the only drop point in this state that a human wrote
        // down. The move is small by construction (NAV hands over within
        // position_tolerance_), and the task_timeout_ below still bounds it.
        target_position_.x = task_reference_.x;
        target_position_.y = task_reference_.y;
        target_position_.z = task_height_;
        ROS_WARN_THROTTLE(1.0,
                          "%s: no stable detection after %.1fs and %.3f m off the "
                          "authored waypoint (tolerance %.2f m); closing that gap "
                          "to drop on (%.2f, %.2f). Abort at %.1f s from the task "
                          "start if it cannot get there",
                          taskName(kind), static_cast<double>(drop_vision_timeout_),
                          distanceToDropWaypoint(),
                          static_cast<double>(drop_waypoint_tolerance_),
                          task_reference_.x, task_reference_.y,
                          static_cast<double>(task_timeout_));
      }
      if (now - detection_started_at_ > task_timeout_)
      {
        abortMission(kind == TASK_HAZARD ? "Hazard target detection timeout"
                                         : "Rescue target detection timeout");
      }
      return;
    }

    const float altitude = std::max(0.2f, static_cast<float>(current_position_.z));
    const float correction_x = pixel_to_body_x_sign_ * last_detection_.pixel_dx * altitude /
                               std::max(1.0f, camera_fx_);
    const float correction_y = pixel_to_body_y_sign_ * last_detection_.pixel_dy * altitude /
                               std::max(1.0f, camera_fy_);
    const float body_correction_x = clampFloat(
        correction_x, -max_task_correction_, max_task_correction_);
    const float body_correction_y = clampFloat(
        correction_y, -max_task_correction_, max_task_correction_);
    // Rotate the correction from the image/body axes into the world frame.
    //
    // The angle here is current_yaw_, the ACTUAL body orientation, not
    // mission_heading_. The camera is hard-mounted to the airframe, so
    // "which way does image-right point in the world" is a question about
    // where the airframe is pointing right now, full stop. mission_heading_ is
    // only the heading we command; if the aircraft has drifted off it, a
    // correction computed against the commanded heading would aim the vehicle
    // at the wrong point on the ground and leave the detector's target
    // off-centre - which is the one thing this path exists to fix.
    //
    // Note the camera plays no part in the position estimate (that is
    // cartographer plus PX4); it only supplies a pixel offset that has to be
    // converted from image axes to world axes. The rotation is purely that
    // axis conversion, and it is unavoidable as long as the camera is fixed to
    // the body rather than gimbal-stabilised in yaw.
    target_position_.x = current_position_.x +
                         std::cos(current_yaw_) * body_correction_x -
                         std::sin(current_yaw_) * body_correction_y;
    target_position_.y = current_position_.y +
                         std::sin(current_yaw_) * body_correction_x +
                         std::cos(current_yaw_) * body_correction_y;
    target_position_.z = task_height_;
    // The correction is now bounded twice: per axis by max_task_correction_, and
    // then as a POINT by the safe field shrunk by the airframe radius. The second
    // one is what actually keeps the aircraft out of a wall - the first alone
    // still allows 0.15 m of travel toward a wall that may be 0.20 m away.
    target_position_ = clampAlignTarget(target_position_);
    task_alignment_started_ = true;
    alignment_started_at_ = now;
    align_entered_at_ = now;
    // The three numbers that make a bad align diagnosable after the fact: what
    // the detector said, what that turned into, and where it was allowed to go.
    ROS_INFO("Align: pixel_d=(%.1f, %.1f) px at %.2f m -> correction=(%.3f, %.3f) m "
             "(clamp %.2f) -> target=(%.2f, %.2f) odom, %.3f m from where the "
             "vehicle is now",
             last_detection_.pixel_dx, last_detection_.pixel_dy,
             static_cast<double>(altitude),
             static_cast<double>(body_correction_x),
             static_cast<double>(body_correction_y),
             static_cast<double>(max_task_correction_),
             target_position_.x, target_position_.y,
             std::hypot(target_position_.x - current_position_.x,
                        target_position_.y - current_position_.y));
    enterState(align_state);
    return;
  }

  if (state_ == align_state)
  {
    // Checked FIRST and measured from entry, never from "arrived". Until
    // 2026-10-17 this branch only refreshed alignment_started_at_ while the
    // vehicle was short of the setpoint, so a setpoint it could not reach was
    // pursued indefinitely - that is the loop that walked the aircraft into the
    // north wall. target_position_ is recomputed as current + correction, so
    // "could not reach" also means "kept trying from a new place each cycle".
    if (align_entered_at_ > 0.0 &&
        now - align_entered_at_ > static_cast<double>(align_timeout_))
    {
      const double missed = std::hypot(target_position_.x - current_position_.x,
                                       target_position_.y - current_position_.y);
      ROS_ERROR("%s align: gave up after %.1f s, %.3f m short of the target "
                "(%.2f, %.2f); alignment tolerance is %.2f m. Not dropping from "
                "here - the payload would land where the position estimate says, "
                "not where the target is. Aborting and returning home.",
                taskName(kind), now - align_entered_at_, missed,
                target_position_.x, target_position_.y,
                static_cast<double>(task_alignment_tolerance_));
      abortMission(kind == TASK_HAZARD ? "Hazard align timeout"
                                       : "Rescue align timeout");
      return;
    }

    if (!atPosition(target_position_, task_alignment_tolerance_, altitude_tolerance_))
    {
      // The one place in the mission that was completely silent while it failed.
      // On 2026-10-18 this state spent its full 5 s and reported only "0.114 m
      // short" - with no way to tell whether the setpoint was moving, whether the
      // vehicle was following it, or whether the position estimate had stopped
      // updating (it had: the node was deadlocked, so current_position_ was
      // frozen at the value it held when the detector's first task-mode frame
      // arrived). Print the two positions and the command, so those three cases
      // are distinguishable from the log alone.
      ROS_INFO_THROTTLE(0.5,
                        "%s align: odom=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f) "
                        "d=%.3f tol=%.2f vel=(%.3f, %.3f) mode=%s armed=%d",
                        taskName(kind), current_position_.x, current_position_.y,
                        current_position_.z, target_position_.x, target_position_.y,
                        target_position_.z,
                        std::hypot(target_position_.x - current_position_.x,
                                   target_position_.y - current_position_.y),
                        static_cast<double>(task_alignment_tolerance_),
                        getVelocity(target_position_.x - current_position_.x, xy_gain_),
                        getVelocity(target_position_.y - current_position_.y, xy_gain_),
                        fcu_state_.mode.c_str(), fcu_state_.armed ? 1 : 0);
      // Only the dwell timer is refreshed here. align_entered_at_ is the one
      // that must survive being off target, or the timeout above never fires.
      alignment_started_at_ = now;
      return;
    }
    if (now - alignment_started_at_ < task_alignment_dwell_)
    {
      return;
    }

    // Which servo command this becomes is decided by beginDrop from the task
    // kind: DROP_FIRE for the hazard material, DROP_RESCUE for the rescue
    // material, each with its own angle in the servo node.
    enterState(drop_state);
  }
}

void RescueMission::beginDrop(TaskKind kind)
{
  drop_started_ = true;
  drop_acknowledged_ = false;
  target_position_.z = task_height_;
  const bool hazard = (kind == TASK_HAZARD);
  sendGpio(hazard ? "DROP_FIRE" : "DROP_RESCUE");
  // Name the parameter that this command actually moves, so a run's two drop
  // angles can be read back out of the log without opening the launch file.
  ROS_INFO("Drop command sent: %s (servo 1 -> %s) %s", hazard ? "DROP_FIRE" : "DROP_RESCUE",
           hazard ? "fire_drop_angle" : "rescue_drop_angle",
           drop_align_mode_ == DROP_ALIGN_WAYPOINT || !task_detection_received_
               ? "[no vision confirmation for this drop]"
               : "");
}

void RescueMission::beginLanding()
{
  setVisionMode(MissionVision::IDLE);
}

void RescueMission::setVisionMode(MissionVision::Mode mode)
{
  vision_mode_ = mode;
  vision_.setMode(mode);

  std_msgs::String message;
  if (mode == MissionVision::HAZARD)
  {
    message.data = "hazard";
  }
  else if (mode == MissionVision::PERSON)
  {
    message.data = "person";
  }
  else
  {
    message.data = "idle";
  }
  vision_mode_pub_.publish(message);
}

void RescueMission::sendGpio(const std::string &command)
{
  std_msgs::String message;
  message.data = command;
  gpio_pub_.publish(message);
}

void RescueMission::requestCameraDown()
{
  const double now = ros::Time::now().toSec();
  sendGpio("CAMERA_DOWN");
  camera_down_requested_ = true;
  camera_down_acknowledged_ = false;
  camera_down_abort_reported_ = false;
  camera_down_started_at_ = now;
  camera_down_last_sent_at_ = now;
}

void RescueMission::requestCameraFront()
{
  const double now = ros::Time::now().toSec();
  sendGpio("CAMERA_FRONT");
  landing_camera_acknowledged_ = false;
  landing_camera_abort_reported_ = false;
  camera_front_started_at_ = now;
  camera_front_last_sent_at_ = now;
}

bool RescueMission::atPosition(const geometry_msgs::Point &target,
                               double xy_tolerance,
                               double z_tolerance) const
{
  // Unlike atRoutePosition, `target` here is already in the ODOM frame: every
  // caller builds it from current_position_ (home_position_, the hover point, a
  // camera-corrected offset from current_position_, ...). So a direct odom
  // comparison is correct and no map->odom conversion belongs here.
  const double dx = current_position_.x - target.x;
  const double dy = current_position_.y - target.y;
  const double dz = current_position_.z - target.z;
  return std::sqrt(dx * dx + dy * dy) <= xy_tolerance && std::fabs(dz) <= z_tolerance;
}

std::string RescueMission::visionWaitReport() const
{
  const int seen = task_frames_seen_.load();
  const int matched = task_frames_matched_.load();
  const double at = task_frame_at_.load();
  const std::string mode = task_kind_ == TASK_HAZARD ? "hazard" : "person";

  std::string report = std::to_string(seen) + " frame(s) seen, " +
                       std::to_string(matched) + " in '" + mode + "' mode";
  if (at <= 0.0)
  {
    // Distinguish the two ways this happens. Both used to read as found=0, and
    // they need opposite fixes: a wrong `mode` field is a mission/vision wiring
    // bug, zero frames is a dead or starved detector - or a hung MissionVision,
    // which is what it was on 2026-10-18 (self-deadlock on detection_mutex_).
    report += seen == 0
                  ? ", none since the task began (nothing is publishing on "
                    "/mission/vision/detection)"
                  : ", none since the task began (every frame carried another mode)";
    return report;
  }
  char age[32];
  std::snprintf(age, sizeof(age), "%.1f", ros::Time::now().toSec() - at);
  report += std::string(", last one ") + age + " s ago";
  return report;
}

double RescueMission::distanceToDropWaypoint() const
{
  return std::hypot(current_position_.x - task_reference_.x,
                    current_position_.y - task_reference_.y);
}

bool RescueMission::atDropWaypoint() const
{
  // Horizontal only, deliberately. The drop is a vertical action from a hover at
  // task_height, and that hover is already enforced separately (the ALIGN state
  // checks the altitude and the HOVER state refuses to start until it is there);
  // folding z into this test would just add a second, tighter altitude gate to
  // the decision that decides whether to drop at all.
  return distanceToDropWaypoint() <= static_cast<double>(drop_waypoint_tolerance_);
}

RescueMission::State RescueMission::hoverState(TaskKind kind) const
{
  return kind == TASK_HAZARD ? HAZARD_HOVER : RESCUE_HOVER;
}

RescueMission::State RescueMission::alignState(TaskKind kind) const
{
  return kind == TASK_HAZARD ? HAZARD_ALIGN : RESCUE_ALIGN;
}

RescueMission::State RescueMission::dropState(TaskKind kind) const
{
  return kind == TASK_HAZARD ? HAZARD_DROP : RESCUE_DROP;
}

const char *RescueMission::taskName(TaskKind kind)
{
  return kind == TASK_HAZARD ? "Hazard" : "Rescue";
}

const char *RescueMission::dropAlignModeName(DropAlignMode mode)
{
  switch (mode)
  {
  case DROP_ALIGN_VISION:
    return "vision";
  case DROP_ALIGN_WAYPOINT:
    return "waypoint";
  case DROP_ALIGN_VISION_FALLBACK:
    return "vision_with_fallback";
  }
  return "unknown";
}

bool RescueMission::atRoutePosition(const geometry_msgs::Point &target,
                                    double xy_tolerance,
                                    double z_tolerance) const
{
  // `target` is the route waypoint. It is now an ODOM coordinate: publishNextGoal
  // stamps the goal header.frame_id = "odom", and current_position_ (PX4/mavros
  // local position) is odom too, so the two are directly comparable and no
  // map->odom correction is involved anywhere.
  //
  // They were not always the same frame. While the goal was published in map,
  // this comparison had to bring map->odom in first. Comparing them raw "worked"
  // only while the offset happened to stay under the tolerance; once it grew
  // past ~0.12 m the mission counted frame offset as navigation error, declared
  // every waypoint unreached, and looped on "Navigation timeout, retry" while
  // DWA dragged the vehicle backwards. Converting at dispatch time (the previous
  // fix) removed the jitter but still depended on cartographer - including on
  // cartographer being alive. Running the stack in odom removes the dependency
  // instead of managing it.
  if (!have_pose_)
  {
    return false;
  }

  geometry_msgs::Point reference = target;
  if (target_odom_valid_)
  {
    reference = target_position_odom_;
  }

  const double dx = current_position_.x - reference.x;
  const double dy = current_position_.y - reference.y;
  const double dz = current_position_.z - reference.z;
  return std::hypot(dx, dy) <= xy_tolerance && std::fabs(dz) <= z_tolerance;
}

void RescueMission::abortMission(const std::string &reason)
{
  ROS_ERROR("Mission aborted: %s", reason.c_str());
  // Tell move_base to drop whatever waypoint it is still chasing. The mission
  // node can only pin its own setpoint; without this preempt the planner keeps
  // publishing velocity for the stale goal and the vehicle flies away. For the
  // ego backend, /move_base/cancel is not subscribed by anything, so it is the
  // /trigger_plan line below that actually stops that planner.
  cancelPlannerGoal();
  setPlanningEnabled(false);
  planner_hold_position_ = current_position_;
  planner_hold_active_ = true;
  landing_requested_ = false;
  if (state_ == LANDING)
  {
    // 降落序列里出错（相机没到位 / PX4 掉了 OFFBOARD）。以前这里进 ABORT 后
    // 只 hold —— 等于"因为降落前的相机确认失败，所以永远不降落"。现在 ABORT
    // 自己会请求 AUTO.LAND，相机只是任务需要，不是降落的前提。
    state_ = ABORT;
    state_started_at_ = ros::Time::now().toSec();
    abort_land_requested_ = false;
    setVisionMode(MissionVision::IDLE);
    ROS_ERROR("Abort during landing: dropping the camera-front requirement; ABORT "
              "will request AUTO.LAND");
    return;
  }
  planner_velocity_ = geometry_msgs::Point();
  planner_command_received_at_ = 0.0;
  planner_motion_received_at_ = 0.0;
  // 状态先摆正，视觉模式最后再切。顺序是刻意的：这一句过去就摆在 here，
  // 于是 2026-10-18 那次 MissionVision 死锁（它那时还持着 mutex_ 才肯放行
  // setVisionMode）把任务线程卡在了"已经打印 abort、但 state_ 还是
  // HAZARD_ALIGN"的地方 —— processMission 的 ABORT 分支永远轮不到，
  // AUTO.LAND 一次都没发出去，飞机就在空中悬到电池结束。任务线程在 abort
  // 路径上不允许有任何东西挡在"让 state_ 变成 ABORT/LANDING"前面。
  if (is_offboard_ && fcu_state_.armed)
  {
    const double home_distance = std::hypot(
        current_position_.x - home_position_.x,
        current_position_.y - home_position_.y);
    if (home_distance <= abort_land_home_radius_)
    {
      ROS_WARN("Abort occurred %.2f m from home; starting the normal landing sequence",
               home_distance);
      enterState(LANDING);
    }
    else
    {
      state_ = ABORT;
      state_started_at_ = ros::Time::now().toSec();
      abort_land_requested_ = false;
      // 2026-10-17 之前这里只有 "Holding at current position; AUTO.LAND not
      // requested"，也就是中止 = 永远悬停。在一个 4 m 见方的场地里，离起飞点
      // 3 m 和 0.3 m 对降落本身没有区别，对"能不能把飞机收回来"却有。
      ROS_ERROR("Abort %.2f m from home: holding for %.1f s, then AUTO.LAND where it "
                "is. It may come down next to an obstacle - that is the trade for not "
                "leaving it hovering until the battery ends.",
                home_distance, static_cast<double>(abort_land_settle_));
    }
  }
  else
  {
    // 没有 OFFBOARD / 没有解锁：没什么可降落的东西。ABORT 分支看到
    // armed=false 会直接进 COMPLETE，所以这里只需要把状态摆正。
    state_ = ABORT;
    state_started_at_ = ros::Time::now().toSec();
    abort_land_requested_ = false;
  }
  // state_ 已经落定，现在才轮到视觉：让检测器别再处理画面，它已经帮不上忙了。
  setVisionMode(MissionVision::IDLE);
}

void RescueMission::cancelPlannerGoal()
{
  // Empty GoalID means "cancel everything" to move_base's action server.
  actionlib_msgs::GoalID cancel;
  cancel.stamp = ros::Time::now();
  goal_cancel_pub_.publish(cancel);
  // Also stop treating the last planner velocity as a live command, so the
  // navigation branch stops believing the planner is still driving us.
  planner_command_received_at_ = 0.0;
  planner_motion_received_at_ = 0.0;
}

void RescueMission::setPlanningEnabled(bool enabled)
{
  if (enabled == planning_enabled_)
  {
    return;
  }
  planning_enabled_ = enabled;
  std_msgs::Bool message;
  message.data = enabled;
  planning_trigger_pub_.publish(message);
  ROS_INFO("Planner %s (/trigger_plan=%s)", enabled ? "enabled" : "disabled",
           enabled ? "true" : "false");
}

bool RescueMission::plannerCommandFresh(double now) const
{
  return planner_command_received_at_ > 0.0 &&
         now - planner_command_received_at_ <= planner_velocity_timeout_;
}

bool RescueMission::visionReadyForTakeoff(double now)
{
  if (!require_vision_ready_)
  {
    return true;
  }

  const double alive_at = vision_alive_at_.load();
  const bool fresh = alive_at > 0.0 && now - alive_at <= 2.0;
  if (fresh)
  {
    if (vision_wait_started_at_ > 0.0)
    {
      // Say how long the wait cost. This is the number that tells the operator
      // whether the gate is protecting them from a 3 s camera start or from a
      // 30 s engine load, and therefore whether the launch order needs fixing.
      ROS_INFO("Vision backend alive after %.1f s of waiting; arming is allowed",
               now - vision_wait_started_at_);
      vision_wait_started_at_ = 0.0;
      vision_wait_reported_ = false;
    }
    return true;
  }

  if (vision_wait_started_at_ <= 0.0)
  {
    vision_wait_started_at_ = now;
  }
  const double waited = now - vision_wait_started_at_;

  if (waited > static_cast<double>(vision_ready_timeout_))
  {
    if (!vision_wait_reported_)
    {
      vision_wait_reported_ = true;
      if (alive_at > 0.0)
      {
        ROS_ERROR("Refusing to take off: the last detection message was %.1f s "
                  "ago and nothing has arrived for the whole %.0f s timeout. The "
                  "vision pipeline died after starting; check the camera and the "
                  "detector before flying. Tighten or bypass deliberately with "
                  "require_vision_ready:=false.",
                  now - alive_at, static_cast<double>(vision_ready_timeout_));
      }
      else
      {
        ROS_ERROR("Refusing to take off: no /mission/vision/detection message "
                  "has EVER arrived (waited %.0f s, timeout %.0f s). The "
                  "detector never produced a frame - engine/model load or camera "
                  "pipeline. auto_start does not fly blind; pass "
                  "require_vision_ready:=false to override on purpose.",
                  waited, static_cast<double>(vision_ready_timeout_));
      }
    }
    return false;
  }

  if (alive_at > 0.0)
  {
    ROS_WARN_THROTTLE(2.0,
                      "Waiting for the vision backend before arming: no "
                      "/mission/vision/detection message for %.1f s (timeout "
                      "%.0f s). The TensorRT engine takes ~26 s to deserialise "
                      "on the Jetson, so this is expected on a cold start.",
                      now - alive_at, static_cast<double>(vision_ready_timeout_));
  }
  else
  {
    ROS_WARN_THROTTLE(2.0,
                      "Waiting for the vision backend before arming: no "
                      "/mission/vision/detection message yet (%.0f s of %.0f s). "
                      "Check that the detector node is running and its camera is "
                      "producing frames.",
                      waited, static_cast<double>(vision_ready_timeout_));
  }
  return false;
}

bool RescueMission::insideSafeField() const
{
  // The measurement stays the odom one; only the bounds' frame is handled here.
  //
  // Why odom: the mission sends setpoints as local (odom) positions, so odom is
  // the frame the vehicle is actually flown in. current_map_position_ comes from
  // SLAM map->base_link and can drift several metres from the truth; testing the
  // field against it aborts the mission for excursions that never happened
  // (observed: map said y=0.61 while the vehicle was at y=-2.30, still near its
  // goal).
  //
  // Why the conversion: safe_field_* are FIELD-frame numbers, and the route
  // frame is the field frame rotated into odom by field_yaw_deg (see the header
  // and applyRouteFrameYaw). At 0 deg this is the identity, so the default
  // behaviour is unchanged.
  const geometry_msgs::Point position = odomToField(current_position_);
  if (!have_pose_ || !std::isfinite(position.x) || !std::isfinite(position.y))
  {
    return false;
  }

  return position.x >= safe_field_min_x_ &&
         position.x <= safe_field_max_x_ &&
         position.y >= safe_field_min_y_ &&
         position.y <= safe_field_max_y_;
}

geometry_msgs::Point RescueMission::clampAlignTarget(
    const geometry_msgs::Point &target)
{
  // Two boxes, two questions:
  //   insideSafeField()  - "may the aircraft be here" (abort envelope, 0.05 m
  //                         from a wall, measured at the CENTRE)
  //   this function      - "may I command it to go here" (setpoint envelope,
  //                         shrunk by the airframe radius, because a setpoint
  //                         takes the rotor tips with it)
  // Conflating them is what let a 0.45 m correction fly a 0.35 m airframe
  // through a wall 0.65 m away on 2026-10-17. The radius used to be configured
  // as 0.22 m, i.e. the tip-to-tip reading of a 450 mm frame, which understated
  // the rotor radius by 13 cm and left that fix short.
  const float min_x = safe_field_min_x_ + align_body_clearance_;
  const float max_x = safe_field_max_x_ - align_body_clearance_;
  const float min_y = safe_field_min_y_ + align_body_clearance_;
  const float max_y = safe_field_max_y_ - align_body_clearance_;

  // A box that has been shrunk past its own width is a configuration error, not
  // something to guess about: pass the target through unchanged and say so
  // loudly, because silently clamping to a degenerate range would fly the
  // vehicle to a corner nobody chose.
  if (max_x <= min_x || max_y <= min_y)
  {
    ROS_ERROR_THROTTLE(2.0,
                       "align_body_clearance=%.2f m does not fit inside the safe "
                       "field (x %.2f..%.2f, y %.2f..%.2f); align targets are NOT "
                       "being clamped. Fix the two parameter sets.",
                       static_cast<double>(align_body_clearance_),
                       static_cast<double>(safe_field_min_x_),
                       static_cast<double>(safe_field_max_x_),
                       static_cast<double>(safe_field_min_y_),
                       static_cast<double>(safe_field_max_y_));
    return target;
  }

  const geometry_msgs::Point field = odomToField(target);
  // 用 double 夹、用 1e-6 判"到底有没有被夹"，**不要**用 float 相等比较。
  // clampFloat 收的是 float，而 field.x 是 double：窄化成 float 再提升回 double
  // 已经不是原值，于是 `clamped_x == field.x` 对任意坐标几乎恒为 false，函数
  // 一路走到 WARN 去报一个 1e-8 m 的"夹取"。2026-10-18 的日志里每一次对齐都
  // 在喊 "would leave the setpoint envelope ... clamped by (0.000, 0.000)" ——
  // 假警报比没有警报更坏：它教人忽略这一行，而这一行本来是 2026-10-17 撞墙之后
  // 加的最后一道提醒（"先看清相机在看什么再相信这次投放"）。
  const double clamped_x = std::min(std::max(field.x, static_cast<double>(min_x)),
                                    static_cast<double>(max_x));
  const double clamped_y = std::min(std::max(field.y, static_cast<double>(min_y)),
                                    static_cast<double>(max_y));
  // 1e-6 m：比任何真实的夹取需求（米级）小六个数量级，比浮点往返噪声大一个量级。
  if (std::fabs(clamped_x - field.x) < 1e-6 &&
      std::fabs(clamped_y - field.y) < 1e-6)
  {
    return target;
  }

  geometry_msgs::Point clamped_field;
  clamped_field.x = clamped_x;
  clamped_field.y = clamped_y;
  clamped_field.z = field.z;
  geometry_msgs::Point clamped = fieldToOdom(clamped_field);
  clamped.z = target.z;

  ROS_WARN("Align target (%.2f, %.2f) odom would leave the setpoint envelope "
           "(x %.2f..%.2f, y %.2f..%.2f in the field frame, i.e. the safe field "
           "shrunk by the %.2f m airframe radius); clamped by (%.3f, %.3f) m. The "
           "visual correction is asking for more room than the field has - check "
           "what the camera is looking at before trusting this drop.",
           target.x, target.y,
           static_cast<double>(min_x), static_cast<double>(max_x),
           static_cast<double>(min_y), static_cast<double>(max_y),
           static_cast<double>(align_body_clearance_),
           clamped.x - target.x, clamped.y - target.y);
  return clamped;
}

float RescueMission::getVelocity(float error, float gain) const
{
  const float value = gain * error;
  return clampFloat(value, -max_velocity_, max_velocity_);
}

const char *RescueMission::stateName(State state)
{
  switch (state)
  {
  case WAIT_FCU:
    return "WAIT_FCU";
  case TAKEOFF:
    return "TAKEOFF";
  case NAV_HAZARD:
    return "NAV_HAZARD";
  case HAZARD_HOVER:
    return "HAZARD_HOVER";
  case HAZARD_ALIGN:
    return "HAZARD_ALIGN";
  case HAZARD_DROP:
    return "HAZARD_DROP";
  case NAV_PERSON:
    return "NAV_PERSON";
  case NAV_RESCUE:
    return "NAV_RESCUE";
  case RESCUE_HOVER:
    return "RESCUE_HOVER";
  case RESCUE_ALIGN:
    return "RESCUE_ALIGN";
  case RESCUE_DROP:
    return "RESCUE_DROP";
  case RETURN_HOME:
    return "RETURN_HOME";
  case LANDING:
    return "LANDING";
  case COMPLETE:
    return "COMPLETE";
  case ABORT:
    return "ABORT";
  }
  return "UNKNOWN";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rescue_mission_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  RescueMission mission(nh, private_nh);
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
