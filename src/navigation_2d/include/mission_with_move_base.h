#pragma once

#include <atomic>
#include <string>
#include <mutex>
#include <vector>

#include <ros/ros.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <navigation_2d/TargetDetection.h>

#include "flight_control.h"
#include "mission_vision.h"

class RescueMission
{
public:
  RescueMission(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  // Leaves the planner switched off. See the note on planning_enabled_.
  ~RescueMission();

private:
  enum State
  {
    WAIT_FCU,
    TAKEOFF,
    NAV_HAZARD,
    HAZARD_HOVER,
    HAZARD_ALIGN,
    HAZARD_DROP,
    NAV_PERSON,
    NAV_RESCUE,
    RESCUE_HOVER,
    RESCUE_ALIGN,
    RESCUE_DROP,
    RETURN_HOME,
    LANDING,
    COMPLETE,
    ABORT
  };

  // Which payload a hover/align/drop phase is delivering.
  //
  // Round 1 delivers the fire/hazard material at the 危险品任务区. Round 2
  // delivers that as well and then the rescue material at a position drawn by
  // lot on the day. The tour of the 人员救助任务区 is a plain route: it needs no
  // camera, so it has no task kind and no hover state of its own any more.
  // (Before 2026-09-30 the person phase also dropped, which is why there used to
  // be PERSON_HOVER/PERSON_ALIGN/PERSON_DROP. That made "fly past every person
  // area" and "drop at a drawn position" the same thing, and they are not.)
  enum TaskKind
  {
    TASK_HAZARD,
    TASK_RESCUE
  };

  // How the drop point is decided. The reasoning behind the default is written
  // out in full in the constructor; the short version is that the camera
  // measures the card and the waypoint only asserts where it is, so the camera
  // is used when it works and the waypoint takes over - but only after checking
  // the vehicle really is over the waypoint - when it does not.
  enum DropAlignMode
  {
    DROP_ALIGN_VISION,          // camera only; no stable detection aborts
    DROP_ALIGN_WAYPOINT,        // authored waypoint only; camera not used
    DROP_ALIGN_VISION_FALLBACK  // camera, then the waypoint (default)
  };

  void stateCallback(const mavros_msgs::State::ConstPtr &msg);
  void servoReadyCallback(const std_msgs::Bool::ConstPtr &msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void plannerVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void gpioAckCallback(const std_msgs::String::ConstPtr &msg);
  void detectionCallback(const navigation_2d::TargetDetection::ConstPtr &msg);
  void controlCallback(const ros::TimerEvent &event);
  void missionCallback(const ros::TimerEvent &event);

  void loadRoute(const std::string &name, std::vector<geometry_msgs::Point> &route);
  void startRoute(const std::vector<geometry_msgs::Point> &route, State state);
  void publishNextGoal();
  void enterState(State state);
  // Decides where the finished route hands over to, and it is the only place
  // `stage` changes the flight: NAV_PERSON ends at RETURN_HOME in round 1 and at
  // NAV_RESCUE in round 2.
  void finishRoute();
  void setVisionMode(MissionVision::Mode mode);
  void sendGpio(const std::string &command);
  void requestCameraDown();
  void requestCameraFront();
  bool atPosition(const geometry_msgs::Point &target, double xy_tolerance, double z_tolerance) const;
  bool routeComplete() const;
  void beginTask(TaskKind kind);
  void processTask(TaskKind kind);
  void beginDrop(TaskKind kind);
  void beginLanding();
  // Distance from the vehicle to the authored drop waypoint of the current task,
  // and the test the fallback uses before it is allowed to drop without a
  // detection. See beginTask for what the waypoint is.
  double distanceToDropWaypoint() const;
  bool atDropWaypoint() const;
  // One-line summary of what /mission/vision/detection did during the current
  // task wait: how many frames arrived, how many of them were addressed to this
  // task's zone, and how long ago the last usable one was. Built for the two
  // timeout messages, which without it cannot separate "the detector says no"
  // from "the vision node stopped talking" (2026-10-18: it was the latter).
  std::string visionWaitReport() const;
  State hoverState(TaskKind kind) const;
  State alignState(TaskKind kind) const;
  State dropState(TaskKind kind) const;
  void abortMission(const std::string &reason);
  void cancelPlannerGoal();
  // Publishes the latched /trigger_plan switch consumed by ego_planner_2d. Idempotent.
  void setPlanningEnabled(bool enabled);
  bool plannerCommandFresh(double now) const;
  // Pre-arm vision gate. True when a detection message arrived within the last
  // 2 s. While it is false, reports why (throttled) and after
  // vision_ready_timeout_ prints one hard error; the caller must not request
  // OFFBOARD or arm in that state. Always true when require_vision_ready_ is
  // false, so a vision-less bench/navigation run stays possible on purpose.
  bool visionReadyForTakeoff(double now);
  bool atRoutePosition(const geometry_msgs::Point &target,
                       double xy_tolerance, double z_tolerance) const;
  bool insideSafeField() const;
  // Keeps a commanded ALIGN setpoint at least align_body_clearance_ inside the
  // safe-field bounds, in the FIELD frame, and returns it back in odom.
  //
  // This is a different question from insideSafeField(): that one asks "where is
  // the aircraft" and allows its centre to reach 0.05 m from a wall, which is a
  // perfectly reasonable abort envelope. This one asks "where am I telling it to
  // go", and a setpoint 0.05 m from a wall puts the rotor tips 0.17 m THROUGH
  // it. Logs once per call when the clamp actually bites, because that means the
  // visual correction wanted to leave the field.
  geometry_msgs::Point clampAlignTarget(const geometry_msgs::Point &target);
  // Rotate the field-frame route tables (and the drawn round-2 drop position)
  // into odom by -field_yaw_deg_. Called once from the constructor, right after
  // the routes are loaded; a no-op at 0.
  void applyRouteFrameYaw();
  // field -> odom and odom -> field for the configured yaw. Pure rotations about
  // the take-off pad, which is the origin of both frames.
  geometry_msgs::Point fieldToOdom(const geometry_msgs::Point &field) const;
  geometry_msgs::Point odomToField(const geometry_msgs::Point &odom) const;
  float getVelocity(float error, float gain) const;
  static const char *stateName(State state);
  static const char *taskName(TaskKind kind);
  static const char *dropAlignModeName(DropAlignMode mode);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher goal_pub_;
  // Preempts move_base on abort. Without this the planner keeps chasing the
  // last waypoint and flies the vehicle out of the field, because the mission
  // node's "hold" only pins its own setpoint and never tells move_base to stop.
  ros::Publisher goal_cancel_pub_;
  ros::Publisher gpio_pub_;
  ros::Publisher vision_mode_pub_;
  // Latched "keep planning" switch for the ego_planner_2d backend.
  //
  // EgoPlanner2DNode has always had this input (trigger_plan ->
  // should_plan_, and planningLoop() returns early while it is false), but no
  // node in the stack ever published it, so should_plan_ stayed at its initial
  // true for the whole run. The planner therefore kept replanning - and kept
  // re-publishing /visual_local_trajectory - after the vehicle had landed and
  // disarmed, which is the observed "it is still planning after it landed".
  //
  // Latched, because the planner may be launched before or after this node: a
  // late subscriber must still be told the current state instead of silently
  // keeping its default.
  ros::Publisher planning_trigger_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber servo_ready_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber planner_velocity_sub_;
  ros::Subscriber gpio_ack_sub_;
  ros::Subscriber detection_sub_;
  ros::Timer control_timer_;
  ros::Timer mission_timer_;
  tf::TransformListener tf_listener_;
  mutable std::mutex map_pose_mutex_;

  FlightControl flight_control_;
  MissionVision vision_;

  mavros_msgs::State fcu_state_;
  geometry_msgs::Point current_position_;
  geometry_msgs::Point current_map_position_;
  double current_yaw_ = 0.0;
  // The heading the aircraft is commanded to hold for the entire flight, in
  // radians, in the odom/local frame. It is NOT the yaw measured at takeoff.
  //
  // It used to be "whatever yaw the vehicle happened to have when the first
  // pose arrived" (mission_heading_ = current_yaw_). That made every consumer
  // of this value depend on one noisy sample taken before the EKF had settled,
  // and it silently assumed the aircraft never rotates afterwards - an
  // assumption nothing in the stack enforced. It is used in two places, both of
  // which inherit any error in it as a fixed rotation:
  //   * as the yaw setpoint of every PositionTarget sent to PX4,
  //   * to rotate the camera-based task correction from body into world.
  // (It used to also rotate /cmd_vel from body into world. That conversion is
  // gone with the body-frame branch in plannerVelocityCallback, 2026-10-17.)
  // A single wrong value therefore steers the whole mission off by a constant
  // angle, which is the observed "drifts even while parked" signature. It is
  // now an explicit mission parameter instead of a measurement.
  double mission_heading_ = 0.0;
  double mission_map_heading_ = 0.0;
  geometry_msgs::Point home_position_;
  geometry_msgs::Point target_position_;
  geometry_msgs::Point planner_velocity_;
  geometry_msgs::Point planner_hold_position_;
  navigation_2d::TargetDetection last_detection_;

  // The active waypoint, in the ODOM frame, i.e. the frame every arrival test
  // and every setpoint uses.
  //
  // History: this field exists because the route tables used to be treated as
  // map-frame coordinates while current_position_ is odom. Cartographer
  // publishes map->odom and that correction is non-zero and moving, so
  // comparing a map-frame target against an odom-frame position silently added
  // the whole offset to the distance: the mission reached the waypoint, measured
  // 0.4 m of "error" that was really frame offset, never registered arrival, and
  // sat re-planning while the controller pulled it backwards.
  //
  // The fix is no longer "convert at dispatch time" - it is that the whole stack
  // runs in odom. publishNextGoal now stamps the goal header.frame_id = "odom"
  // and the route tables are read as odom coordinates, which is legitimate
  // because px4_sensor_rig_node makes odom the Gazebo ground truth (exact, no
  // drift) and its origin coincides with the field frame the routes are authored
  // in. So this is a copy of target_position_, and both are odom.
  // target_odom_valid_ is retained only to keep the NAV cmd log's last field
  // meaningful; it is always true once a waypoint has been dispatched.
  geometry_msgs::Point target_position_odom_;
  bool target_odom_valid_ = false;

  std::vector<geometry_msgs::Point> hazard_route_;
  std::vector<geometry_msgs::Point> person_route_;
  std::vector<geometry_msgs::Point> return_route_;
  // One-point route to the drawn round-2 drop position. A route rather than a
  // bare point because it is flown by exactly the same NAV_* machinery as the
  // other three, including the arrival test and the planner stall recovery, and
  // because the operator can add an approach point here if the drawn position
  // ends up behind an obstacle on the day.
  std::vector<geometry_msgs::Point> rescue_drop_route_;
  const std::vector<geometry_msgs::Point> *active_route_ = nullptr;
  std::size_t route_index_ = 0;

  State state_ = WAIT_FCU;
  MissionVision::Mode vision_mode_ = MissionVision::IDLE;
  bool have_pose_ = false;
  bool have_map_pose_ = false;
  bool origin_offset_logged_ = false;
  bool have_map_heading_ = false;
  bool have_home_ = false;
  bool servo_ready_ = false;
  bool require_servo_ready_ = true;
  // Vision liveness gate, checked before the vehicle is allowed to arm.
  //
  // auto_start used to depend on exactly one "peripheral is up" signal - the
  // servo's latched /servo_ready - plus 100 setpoints (2 s at 50 Hz). The
  // detector was not in that chain at all, and on the Jetson the TensorRT
  // engine takes ~26 s to deserialise with the CPU saturated, so the vehicle
  // could be airborne half a minute before the first frame existed. That is the
  // documented aggravating factor of the 2026-10-07 crash (cartographer
  // starving during takeoff) and it also means the task phase starts with no
  // detections at all.
  //
  // "Ready" is deliberately defined as "a detection message arrived recently"
  // rather than as a new /ready topic: /mission/vision/detection is published
  // once per processed frame by BOTH backends (found=false included, and the
  // ssd node sends one message per frame with an empty array as heartbeat), so
  // it is already an end-to-end liveness signal covering the camera, the
  // engine and the whole message path. A dedicated topic would prove less.
  bool require_vision_ready_ = true;
  float vision_ready_timeout_ = 60.0f;
  // Written from the detection callback, read from the mission timer, and this
  // node runs under an AsyncSpinner(3), so this scalar really does cross
  // threads. atomic<double> instead of a mutex: one value, no invariant.
  std::atomic<double> vision_alive_at_{0.0};
  double vision_wait_started_at_ = 0.0;
  bool vision_wait_reported_ = false;
  // A route leg that never produced a single MOVING planner command within this
  // many seconds is dead, and re-sending the same goal cannot revive it: when
  // EGO refuses a goal (start or goal inside the inflated map) every retry
  // fails identically. 2026-10-17: the vehicle sat ~0.15 m short of
  // hazard_route[1] through 40 identical replans while this node waited out
  // planner_failure_abort_timeout_ twice. Must stay below that timeout so the
  // specific reason is reported before the generic "Planner command timeout".
  float route_no_motion_timeout_ = 6.0f;
  bool auto_start_ = true;
  bool rescue_drop_enabled_ = false;
  bool is_offboard_ = false;
  bool drop_started_ = false;
  bool drop_acknowledged_ = false;
  bool landing_requested_ = false;
  // ABORT 之后不能停在这里。settle 一下再让 PX4 接管降落，并记住"已经请求
  // 过"，否则 controlCallback 会继续发 hold setpoint 和 AUTO.LAND 抢高度。
  bool abort_land_requested_ = false;
  bool task_detection_received_ = false;
  // Frame accounting for the task wait. "no stable detection (last: found=0)"
  // cannot distinguish three failures that need completely different fixes:
  //   * 0 frames seen           -> nothing upstream is publishing. Every filter
  //                                inside MissionVision is irrelevant; the
  //                                problem is the detector, the camera, or a
  //                                stalled/hung node;
  //   * frames seen, 0 matched  -> messages arrive but their `mode` field is not
  //                                the one this task waits for, and
  //                                detectionCallback returns before recording
  //                                anything. Silent until now, and it is a whole
  //                                class of bug on its own;
  //   * frames matched, found=0 -> the detector really reported nothing, and the
  //                                answer is inside MissionVision (zone, score,
  //                                frame-edge clipping).
  // 2026-10-18: the first of those was happening and the log looked identical to
  // the third. Written from the detection callback, read from the mission timer
  // under AsyncSpinner(3), so one atomic each - no invariant to protect.
  std::atomic<int> task_frames_seen_{0};
  std::atomic<int> task_frames_matched_{0};
  std::atomic<double> task_frame_at_{0.0};
  // How long a silent /mission/vision/detection streak may last before the task
  // phase says so out loud. 2 s is several frames at any plausible detector rate
  // and far below drop_vision_timeout, so the operator sees "the vision node went
  // quiet" instead of only "no stable detection" five seconds later.
  float vision_silence_timeout_ = 2.0f;
  bool task_alignment_started_ = false;
  TaskKind task_kind_ = TASK_HAZARD;
  // The authored drop waypoint of the current task, in odom: the last
  // hazard_route point for the hazard drop, the drawn position for the rescue
  // drop. Used by atDropWaypoint() and by the "where did it actually drop"
  // logging, so that a fallback drop is never confused with an aligned one.
  geometry_msgs::Point task_reference_;
  // The drawn round-2 drop position, and the route that flies to it. Built once
  // in the constructor; have_rescue_drop_point_ is false when the parameter was
  // not supplied, which is a refusal to start rather than a default.
  geometry_msgs::Point rescue_drop_point_;
  bool have_rescue_drop_point_ = false;
  // 1 = 第一轮 (hazard drop only), 2 = 第二轮 (hazard drop + rescue-material
  // drop). Reconciled with rescue_drop_enabled_ in the constructor so the two
  // inputs cannot disagree.
  int stage_ = 1;
  DropAlignMode drop_align_mode_ = DROP_ALIGN_VISION_FALLBACK;
  bool planner_hold_active_ = false;
  bool camera_down_requested_ = false;
  bool camera_down_acknowledged_ = false;
  bool camera_down_abort_reported_ = false;
  bool landing_camera_acknowledged_ = false;
  bool landing_camera_abort_reported_ = false;
  bool landing_failed_ = false;
  bool enforce_safe_field_ = false;
  bool use_position_hint_ = true;
  bool abort_on_offboard_loss_ = true;
  // Last value written to /trigger_plan. Starts true only so that the
  // constructor's setPlanningEnabled(false) is not swallowed as a no-op.
  bool planning_enabled_ = true;
  int preflight_setpoints_ = 0;
  int navigation_retries_ = 0;

  double state_started_at_ = 0.0;
  double route_goal_started_at_ = 0.0;
  double detection_started_at_ = 0.0;
  double alignment_started_at_ = 0.0;
  // When ALIGN was entered. Distinct from alignment_started_at_ on purpose:
  // that one is the dwell timer and is reset while the vehicle is short of the
  // setpoint, while this one must never be reset or the align timeout is dead.
  double align_entered_at_ = 0.0;
  double last_mode_request_at_ = 0.0;
  double last_arm_request_at_ = 0.0;
  double abort_land_requested_at_ = 0.0;
  double last_gpio_ack_at_ = 0.0;
  double camera_down_started_at_ = 0.0;
  double camera_down_last_sent_at_ = 0.0;
  double camera_down_ack_at_ = 0.0;
  double camera_front_started_at_ = 0.0;
  double camera_front_last_sent_at_ = 0.0;
  double landing_camera_ack_at_ = 0.0;
  double planner_command_received_at_ = 0.0;
  double planner_motion_received_at_ = 0.0;

  float fly_height_ = 1.5f;
  float task_height_ = 1.2f;
  // 起飞段发给 PX4 的高度。它只在这段里被自己的斜坡抬到 fly_height_，不参与
  // 悬停/对齐。为什么起飞必须和别的阶段分开，见 controlCallback 里 TAKEOFF
  // 那一支的注释：起飞的 XY 目标是常数，不需要"设定点跟着实测走"的软启动，
  // 而这个软启动正是把估计延迟变成横向晃动的东西（2026-10-18，仿真同样晃）。
  // 进入 TAKEOFF 时取 min(当前 z, fly_height_)，所以从高处手动起任务也能往下收。
  double takeoff_cmd_z_ = 0.0;
  // Mission heading, in degrees, in the odom/local frame. 0 deg means the nose
  // points along +x. See the note on mission_heading_ above: this is a fixed
  // mission constant, deliberately not derived from the takeoff pose. Exposed
  // as a parameter so a run can be reproduced (and so a future change to the
  // spawn yaw is a config change, not a measurement).
  float fix_heading_deg_ = 0.0f;
  // Heading of the airframe in the FIELD frame, in degrees, at the moment
  // cartographer started tracking - i.e. the angle between the odom +x axis and
  // the field +x axis. 0 = nose along the field +x, 90 = +y, 180 = -x, -90 = -y.
  //
  // Why it is needed: the route tables in navigation_2d.launch are FIELD-frame
  // numbers (the drawing's millimetres minus the origin shift), while the goals
  // this node publishes are stamped "odom". Those two frames coincide only in
  // the simulation, where px4_sensor_rig_node is the sole publisher of
  // odom -> base_link and derives it from Gazebo ground truth, and where the
  // iris spawns at the field origin with zero yaw. On the vehicle odom is
  // cartographer's own frame (location/param/ROSSLAM/
  // cartographer_2Dlidar_mapping_location_params.lua: provide_odom_frame=true,
  // published_frame=base_link, use_odometry=false), so its yaw is whatever
  // heading the airframe had when tracking started, and nothing absolute pins
  // it. This parameter is the one place that fact is written down: every route
  // point is rotated by -field_yaw_deg on the way out, so the launch file keeps
  // describing the field and the default 0.0 changes nothing.
  //
  // Consequence worth knowing before flying: the measured odom yaw is 0 at boot
  // BY CONSTRUCTION (cartographer's local pose starts at identity and PX4 takes
  // the vision yaw as its reference), so a wrong value here cannot be caught by
  // reading the yaw back. It is caught by looking at the map: the field's
  // bounding box in odom is printed at startup and it is a different rectangle
  // for each of 0 / +-90 / 180 deg, because the pad sits at a corner of the
  // field rather than at its centre.
  float field_yaw_deg_ = 0.0f;
  double field_yaw_rad_ = 0.0;
  float xy_gain_ = 0.8f;
  float z_gain_ = 1.0f;
  float max_velocity_ = 0.35f;
  float position_tolerance_ = 0.12f;
  float altitude_tolerance_ = 0.08f;
  // Altitude window used while flying a route. The mission node holds z at
  // fly_height itself (see processNavigation: planner_position.z = fly_height_),
  // so move_base/DWA only ever drives x/y. Judging "waypoint reached" with the
  // tight altitude_tolerance_ made the state machine declare every waypoint
  // unreached the moment DWA reported "Goal reached" and cut its velocity,
  // which then looked like a stall and re-sent the same goal in a loop.
  // Keep this generous: horizontal accuracy is what matters en route.
  float route_altitude_tolerance_ = 0.40f;
  float task_alignment_tolerance_ = 0.08f;
  float task_pixel_to_meter_scale_ = 1.0f;
  float camera_fx_ = 874.0f;
  float camera_fy_ = 874.0f;
  float pixel_to_body_x_sign_ = 1.0f;
  float pixel_to_body_y_sign_ = 1.0f;
  // Largest horizontal correction a single ALIGN command may ask for. 0.45 m
  // was the value that flew the aircraft into the north wall on 2026-10-17: the
  // hazard drop point is only 0.65 m from that wall and the airframe is 0.35 m
  // from centre to rotor tip (450 mm axis with 10-inch props; it was configured
  // as 0.22 m then, which is the tip-to-tip reading and 13 cm too small).
  // 0.15 m converges over several 50 Hz cycles because the target is
  // recomputed as current + correction every cycle, so a large offset is walked
  // down rather than jumped.
  float max_task_correction_ = 0.15f;
  // Total time allowed in the ALIGN state, measured from entry and NOT reset by
  // "not there yet". Before this existed the align branch reset its own timer
  // every cycle it was short of the target, so a setpoint the vehicle could not
  // reach was pursued forever at 50 Hz. Abort, never a silent drop.
  float align_timeout_ = 5.0f;
  // Airframe radius, centre to rotor tip. Also read by tools/check_config.py as
  // the lower bound for map_inflate_value.
  // 0.35 = 450 mm 轴距 / 2 + 10 寸桨半径 0.127 (2026-10-17). The old 0.22 was
  // the tip-to-tip reading of "450 mm frame" and understated the rotor radius by
  // 13 cm, which is why the 2026-10-17 align clamp did not actually keep the
  // tips inside the wall.
  float align_body_clearance_ = 0.35f;
  float waypoint_dwell_ = 0.6f;
  float camera_settle_time_ = 0.8f;
  float task_timeout_ = 12.0f;
  float task_alignment_dwell_ = 0.7f;
  // How long the camera gets before a fallback-mode drop gives up on it, and
  // how close the vehicle must already be to the authored waypoint for that
  // fallback to be allowed at all. Must stay below task_timeout_, which is the
  // point at which even the fallback stops waiting.
  float drop_vision_timeout_ = 5.0f;
  float drop_waypoint_tolerance_ = 0.20f;
  float drop_settle_time_ = 1.5f;
  float navigation_timeout_ = 30.0f;
  float planner_velocity_timeout_ = 0.6f;
  // Must stay in step with the param default in rescue_mission.cpp and the
  // value in navigation_2d.launch. DWA publishes zero velocity whenever
  // transformGlobalPlan() fails (map -> odom extrapolation) or it reports
  // "failed to produce path", and planner_motion_received_at_ is only
  // refreshed above 0.03 m/s - so this timer alone decides whether a
  // recoverable local-planning hiccup aborts the mission.
  float planner_failure_abort_timeout_ = 10.0f;
  float camera_command_timeout_ = 5.0f;
  float camera_command_retry_interval_ = 1.5f;
  float landing_camera_post_move_settle_ = 0.3f;
  float camera_down_post_move_settle_ = 0.3f;
  float landing_timeout_ = 20.0f;
  float safe_field_min_x_ = -0.60f;
  float safe_field_max_x_ = 3.30f;
  float safe_field_min_y_ = -3.30f;
  // The vehicle spawns at map y ~= 0, only ~0.45 m from the old 0.45 limit,
  // so a few centimetres of takeoff jitter aborted the mission instantly.
  // The enclosure inner face is y = 0.65, so allow most of that margin.
  float safe_field_max_y_ = 0.60f;
  float abort_land_home_radius_ = 0.25f;
  // 中止后到请求 AUTO.LAND 之间的原地保持时间，以及"模式没切过去"时的重发
  // 间隔。含义见 abortMission / missionCallback 里 case ABORT 的注释。
  float abort_land_settle_ = 0.5f;
  float abort_land_retry_interval_ = 2.0f;
  int max_navigation_retries_ = 2;
};
