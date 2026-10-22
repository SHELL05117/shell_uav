// Standalone format-string check for the four log calls added on 2026-09-25.
// GCC's format(printf) attribute makes the compiler verify the format string
// against the argument list exactly as ROS's macros do downstream, without
// needing any ROS headers.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace ros_stub
{
__attribute__((format(gnu_printf, 2, 3))) void warn_throttle(double, const char *, ...);
__attribute__((format(gnu_printf, 2, 3))) void info_throttle(double, const char *, ...);
__attribute__((format(gnu_printf, 2, 3))) void error_throttle(double, const char *, ...);
__attribute__((format(gnu_printf, 1, 2))) void info(const char *, ...);
__attribute__((format(gnu_printf, 1, 2))) void warn(const char *, ...);
__attribute__((format(gnu_printf, 1, 2))) void error(const char *, ...);
__attribute__((format(gnu_printf, 1, 2))) void fatal(const char *, ...);
}  // namespace ros_stub

#define ROS_WARN_THROTTLE(period, ...) ros_stub::warn_throttle(period, __VA_ARGS__)
#define ROS_INFO_THROTTLE(period, ...) ros_stub::info_throttle(period, __VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...) ros_stub::error_throttle(period, __VA_ARGS__)
#define ROS_WARN(...) ros_stub::warn(__VA_ARGS__)
#define ROS_INFO(...) ros_stub::info(__VA_ARGS__)
#define ROS_ERROR(...) ros_stub::error(__VA_ARGS__)
#define ROS_FATAL(...) ros_stub::fatal(__VA_ARGS__)

struct PathPoint
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double v = 0.0;
};

struct ObstacleInfo
{
  double x = 0.0;
  double y = 0.0;
};

struct Pose
{
  PathPoint position;
};

struct PoseStamped
{
  Pose pose;
};

struct Path
{
  std::vector<PoseStamped> poses;
};

static double distance(const PathPoint &, const PathPoint &) { return 0.0; }

void check_ego_replan(const std::vector<PathPoint> &global_path,
                      const std::vector<PathPoint> &ref_path,
                      const std::vector<PathPoint> &planned_traj_,
                      const std::vector<ObstacleInfo> &local_obstacles,
                      const std::vector<std::vector<int> > &a_star_pathes_,
                      size_t discretized_size,
                      double path_sample_interval_)
{
  if (discretized_size < 2)
  {
    ROS_WARN_THROTTLE(1.0,
                      "EGO replan skipped: reference path discretized to %zu "
                      "point(s) at %.2f m spacing (global path has %zu point(s))",
                      discretized_size, path_sample_interval_,
                      global_path.size());
  }

  const PathPoint window_end = ref_path.back();
  const PathPoint goal_point = global_path.back();
  const bool window_end_inflated = true;
  const bool goal_inflated = false;
  const double window_end_to_goal = distance(window_end, goal_point);

  if (planned_traj_.empty())
  {
    ROS_WARN_THROTTLE(1.0,
                      "EGO spline optimization and A* fallback produced no safe "
                      "trajectory: %zu ref point(s), window end (%.2f, %.2f) is "
                      "%.2f m from the goal (%.2f, %.2f); window_end_inflated=%d "
                      "goal_inflated=%d obstacles=%zu a_star_paths=%zu",
                      ref_path.size(), window_end.x, window_end.y,
                      window_end_to_goal, goal_point.x, goal_point.y,
                      window_end_inflated ? 1 : 0, goal_inflated ? 1 : 0,
                      local_obstacles.size(), a_star_pathes_.size());
  }
  else
  {
    const PathPoint planned_end = planned_traj_.back();
    const size_t inflated_pts = 0;
    ROS_INFO_THROTTLE(2.0,
                      "EGO planned %zu point(s), end (%.2f, %.2f) is %.3f m "
                      "from the goal (%.2f, %.2f); window_end_inflated=%d "
                      "goal_inflated=%d inflated_pts=%zu obstacles=%zu",
                      planned_traj_.size(), planned_end.x, planned_end.y,
                      distance(planned_end, goal_point),
                      goal_point.x, goal_point.y,
                      window_end_inflated ? 1 : 0, goal_inflated ? 1 : 0,
                      inflated_pts, local_obstacles.size());
  }
}

void check_controller(const Path &trajectory, double x, double y,
                      double goal_tolerance_)
{
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
}

// ---------------------------------------------------------------------------
// Added later on 2026-09-25, while fixing the EGO "-1021 then freeze" chain.
// %zu must match std::size_t, %d must match int - note that Eigen::Index is NOT
// an int, which is why every cols() below is cast.
// ---------------------------------------------------------------------------
void check_ego_replan_v2(const std::vector<PathPoint> &global_path,
                         const std::vector<PathPoint> &ref_path,
                         const std::vector<PathPoint> &planned_traj_,
                         const std::vector<ObstacleInfo> &local_obstacles,
                         size_t discretized_size, double path_sample_interval_,
                         double span, double tighter, size_t min_ref_points_)
{
  if (discretized_size < 2)
  {
    ROS_WARN_THROTTLE(1.0,
                      "EGO replan skipped: reference path discretized to %zu "
                      "point(s) at %.2f m spacing (global path has %zu point(s))",
                      discretized_size, path_sample_interval_,
                      global_path.size());
  }

  std::vector<PathPoint> dense = ref_path;
  if (ref_path.size() < min_ref_points_)
  {
    ROS_INFO_THROTTLE(
        2.0,
        "EGO re-sampled a short reference path: %zu point(s) over %.3f m at "
        "%.3f m spacing -> %zu point(s) (optimizer needs > 4).",
        ref_path.size(), span, tighter, dense.size());
  }

  const PathPoint window_end = ref_path.back();
  const PathPoint goal_point = global_path.back();
  const bool window_end_inflated = true;
  const bool goal_inflated = false;
  const double window_end_to_goal = distance(window_end, goal_point);

  if (planned_traj_.empty())
  {
    ROS_WARN_THROTTLE(1.0,
                      "EGO spline optimization and A* fallback produced no safe "
                      "trajectory: %zu ref point(s), window end (%.2f, %.2f) is "
                      "%.2f m from the goal (%.2f, %.2f); window_end_inflated=%d "
                      "goal_inflated=%d obstacles=%zu a_star_paths=%zu",
                      ref_path.size(), window_end.x, window_end.y,
                      window_end_to_goal, goal_point.x, goal_point.y,
                      window_end_inflated ? 1 : 0, goal_inflated ? 1 : 0,
                      local_obstacles.size(), global_path.size());

    if (planned_traj_.empty())
      ROS_WARN_THROTTLE(1.0, "EGO has no previous trajectory to fall back on.");
    else
      ROS_WARN_THROTTLE(1.0,
                        "EGO keeping the previous %zu-point trajectory "
                        "(its far end is what the controller will stop at).",
                        planned_traj_.size());
    return;
  }
  else
  {
    const size_t planned_size = planned_traj_.size();
    const PathPoint planned_end = planned_traj_.back();
    const size_t inflated_pts = 0;
    ROS_INFO_THROTTLE(2.0,
                      "EGO planned %zu point(s), end (%.2f, %.2f) is %.3f m "
                      "from the goal (%.2f, %.2f); window_end_inflated=%d "
                      "goal_inflated=%d inflated_pts=%zu obstacles=%zu",
                      planned_size, planned_end.x, planned_end.y,
                      distance(planned_end, goal_point), goal_point.x,
                      goal_point.y, window_end_inflated ? 1 : 0,
                      goal_inflated ? 1 : 0, inflated_pts,
                      local_obstacles.size());
  }
}

void check_makePlan_dispatch(size_t point_set_size, size_t kMinPointsForBspline,
                             int ctrl_pts_cols, int bspline_order,
                             double end_to_goal, double start_to_goal)
{
  printf("Reference path has only %zu point(s); the B-spline optimizer "
         "needs at least %zu. Dispatching straight to the A* fallback.\n",
         point_set_size, kMinPointsForBspline);
  printf("B-spline parameterization produced only %d control point(s) "
         "(need > %d); skipping the optimizer.\n",
         ctrl_pts_cols, 2 * bspline_order);
  printf("A* fallback ends %.3f m from the goal, no better than "
         "the current %.3f m; discarding.\n",
         end_to_goal, start_to_goal);
}

void check_optimizer_guards(int cps_cols, int order)
{
  printf("BsplineOptimizer: reference path too short (%d control point(s), "
         "need > 2 * order = %d) - skipping the B-spline optimizer.\n",
         cps_cols, 2 * order);
  printf("BsplineOptimizer (refine): trajectory too short (%d control "
         "point(s), need > 2 * order = %d) - skipping refinement.\n",
         cps_cols, 2 * order);
}

// ---------------------------------------------------------------------------
// Added 2026-09-25 (later) while fixing "still planning after landing" and the
// left-right sway. Both new log lines carry floats, so they belong here: a
// mismatch between %.2f and a std::size_t (or the reverse) is exactly the kind of
// defect that only shows up as garbage in a log, or as a crash on a 64-bit build.
//
// 2026-10-17: the cmd_vel_frame / output_frame switches were deleted, so the
// three call sites that took them as arguments are gone too. What replaced them
// is a single fixed sentence on each side. Keep those here - a constant string
// is still a string that gets printf-checked, and the day somebody re-adds a
// %s to it is exactly the day this file earns its keep.
// ---------------------------------------------------------------------------
void check_cmd_vel_frame_notice()
{
  ROS_INFO("cmd_vel is read as an odom-frame velocity (single rotation in the "
           "whole chain, no body->world conversion here)");
}

void check_ego_ready(const char *trajectory, const char *tf_parent,
                     const char *tf_child, const char *cmd_vel,
                     const char *trigger)
{
  ROS_INFO("EGO trajectory controller ready: trajectory=%s TF=%s->%s "
           "command=%s (odom frame, no rotation) trigger=%s",
           trajectory, tf_parent, tf_child, cmd_vel, trigger);
}

void check_planning_switch(const char *state, int route_index, size_t route_size)
{
  ROS_INFO("Planner %s (/trigger_plan=%s)", "disabled", "false");
  ROS_INFO("Planning is disabled; cached plan dropped and an empty trajectory "
           "published. The follower will hold until /trigger_plan goes true.");
  ROS_INFO("EGO follower: planning %s", "disabled");
  ROS_INFO_THROTTLE(5.0, "EGO follower: planning disabled, commanding zero velocity");
  (void)state;
  (void)route_index;
  (void)route_size;
}

void check_cmd_vel_forwarding(double in_x, double in_y, double out_x, double out_y)
{
  ROS_INFO_THROTTLE(1.0,
                    "cmd_vel: in=(%.3f, %.3f) -> out=(%.3f, %.3f) "
                    "[odom passthrough, no rotation]",
                    in_x, in_y, out_x, out_y);
}

void check_follower_trace(double x, double y, double yaw_deg, double lookahead,
                          double target_x, double target_y, double dx, double dy,
                          double cross_track, double remaining, double speed,
                          double cmd_x, double cmd_y)
{
  ROS_INFO_THROTTLE(1.0,
                    "EGO follower: pose=(%.2f, %.2f) yaw=%.1f deg lookahead=%.2f "
                    "target=(%.2f, %.2f) delta=(%.3f, %.3f) lag=%.3f rem=%.2f "
                    "speed=%.3f cmd=(%.3f, %.3f) [odom]",
                    x, y, yaw_deg, lookahead, target_x, target_y, dx, dy,
                    cross_track, remaining, speed, cmd_x, cmd_y);
}

// ---------------------------------------------------------------------------
// Added 2026-09-30 for the two-competition-round change: `stage`, the drawn
// rescue drop position, and the vision/waypoint drop modes.
//
// Two things in here are worth noticing on purpose:
//   * every argument that used to be a float is compared against a "%.Nf" as it
//     is in the source, because varargs promotes float to double and a mismatch
//     would only show up as garbage in a log;
//   * every "%.2f" that feeds a value the node stores as float is written here
//     with the same static_cast<double> the source uses, so this file fails if
//     someone later changes one side and not the other.
// ---------------------------------------------------------------------------
struct TargetDetectionStub
{
  bool found = false;
  bool stable = false;
};

enum TaskKindStub
{
  TASK_HAZARD_STUB,
  TASK_RESCUE_STUB
};

static const char *taskNameStub(TaskKindStub kind)
{
  return kind == TASK_HAZARD_STUB ? "Hazard" : "Rescue";
}

static const char *dropAlignModeNameStub() { return "vision_with_fallback"; }

void check_stage_options(int stage, const std::string &drop_align_mode,
                         float drop_vision_timeout, float drop_waypoint_tolerance,
                         float task_timeout, float planner_velocity_timeout,
                         double rescue_drop_x, double rescue_drop_y,
                         float safe_field_min_x, float safe_field_max_x,
                         float safe_field_min_y, float safe_field_max_y)
{
  ROS_WARN("stage %d is neither 1 nor 2; flying this run as stage 1", stage);
  ROS_WARN("rescue_drop_enabled=true but stage=%d; the rescue drop only exists "
           "in round 2, so this run is treated as stage 2", stage);
  ROS_WARN("drop_align_mode '%s' is not 'vision', 'waypoint' or "
           "'vision_with_fallback'; using vision_with_fallback",
           drop_align_mode.c_str());
  ROS_WARN("drop_vision_timeout=%.1fs is not shorter than task_timeout=%.1fs; "
           "the fallback would never get a chance before the abort",
           static_cast<double>(drop_vision_timeout),
           static_cast<double>(task_timeout));
  ROS_INFO("Rescue mission initialized. auto_start=%s planner_timeout=%.2fs",
           "true", static_cast<double>(planner_velocity_timeout));
  ROS_INFO("Competition stage %d: %s", stage,
           stage >= 2 ? "hazard drop + rescue-material drop"
                      : "hazard drop only, no rescue-material drop");
  ROS_INFO("Drop alignment mode: %s (%s)", dropAlignModeNameStub(),
           drop_align_mode == "vision_with_fallback"
               ? "camera first, authored waypoint as the fallback"
               : "other");
  ROS_INFO("Drop fallback: after %.1fs without a stable detection, drop on the "
           "authored waypoint if the vehicle is within %.2f m of it",
           static_cast<double>(drop_vision_timeout),
           static_cast<double>(drop_waypoint_tolerance));
  ROS_INFO("Rescue-material drop waypoint: (%.2f, %.2f) in odom, flown after "
           "every person task area", rescue_drop_x, rescue_drop_y);
  ROS_FATAL("stage %d needs rescue_drop_x and rescue_drop_y inside the field "
            "(x %.2f..%.2f, y %.2f..%.2f); got (%.2f, %.2f). Refusing to take "
            "off - set both on the command line to the drawn position.",
            stage, static_cast<double>(safe_field_min_x),
            static_cast<double>(safe_field_max_x),
            static_cast<double>(safe_field_min_y),
            static_cast<double>(safe_field_max_y), rescue_drop_x, rescue_drop_y);
}

void check_stage_gate(int stage, float safe_field_min_x, float safe_field_max_x,
                      float safe_field_min_y, float safe_field_max_y)
{
  ROS_ERROR_THROTTLE(2.0,
                     "Stage %d is configured without a usable rescue drop "
                     "waypoint; not starting. Pass rescue_drop_x and "
                     "rescue_drop_y inside the field (x %.2f..%.2f, y %.2f..%.2f).",
                     stage, static_cast<double>(safe_field_min_x),
                     static_cast<double>(safe_field_max_x),
                     static_cast<double>(safe_field_min_y),
                     static_cast<double>(safe_field_max_y));
}

void check_route_handover(int stage, double rescue_drop_x, double rescue_drop_y)
{
  ROS_INFO("Person task areas complete; flying to the drawn rescue drop "
           "position (%.2f, %.2f)", rescue_drop_x, rescue_drop_y);
  ROS_INFO("Person task areas complete; returning home (stage %d has no "
           "rescue-material drop)", stage);
}

void check_task_begin(TaskKindStub kind, float task_height, double ref_x,
                      double ref_y)
{
  ROS_INFO("%s task: hover at %.2f m, drop reference waypoint (%.2f, %.2f), "
           "align mode %s",
           taskNameStub(kind), static_cast<double>(task_height), ref_x, ref_y,
           dropAlignModeNameStub());
  ROS_WARN("hazard_route is empty, so the hazard drop has no reference "
           "waypoint; the waypoint fallback is unavailable for this drop");
}

void check_drop_decision(TaskKindStub kind, double distance_to_waypoint,
                         float drop_waypoint_tolerance, float drop_vision_timeout,
                         const TargetDetectionStub &last_detection, double ref_x,
                         double ref_y, float task_timeout)
{
  ROS_WARN_THROTTLE(1.0,
                    "%s waypoint drop: %.3f m from the reference waypoint "
                    "(%.2f, %.2f), tolerance %.2f m; holding instead of dropping",
                    taskNameStub(kind), distance_to_waypoint, ref_x, ref_y,
                    static_cast<double>(drop_waypoint_tolerance));
  ROS_INFO("%s drop: waypoint mode, vision not used; dropping on (%.2f, %.2f) "
           "from %.3f m",
           taskNameStub(kind), ref_x, ref_y, distance_to_waypoint);
  ROS_WARN("%s: no stable detection after %.1fs (last: found=%d stable=%d; %s); "
           "falling back to the authored waypoint (%.2f, %.2f), %.3f m away. "
           "The payload lands where the position estimate says, not where "
           "the target is - check the video before trusting this drop.",
           taskNameStub(kind), static_cast<double>(drop_vision_timeout),
           last_detection.found ? 1 : 0, last_detection.stable ? 1 : 0,
           "0 frame(s) seen, 0 in 'hazard' mode, none since the task began",
           ref_x, ref_y, distance_to_waypoint);
  // 2026-10-18: 沉默看门狗与 ALIGN 进度。两条都是多参数格式串，且都在
  // "看不到就完全查不出原因"的位置上，所以它们必须在桩里被 g++ 的类型检查过。
  ROS_WARN_THROTTLE(1.0,
                    "%s: no /mission/vision/detection frame for %.1fs (%s). "
                    "Boxes still drawn on /ssd_detection/image/compressed do "
                    "not contradict this: that image is produced inside the "
                    "detector node, upstream of every filter here.",
                    taskNameStub(kind), 3.5,
                    "0 frame(s) seen, 0 in 'hazard' mode, none since the task began");
  ROS_INFO_THROTTLE(0.5,
                    "%s align: odom=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f) "
                    "d=%.3f tol=%.2f vel=(%.3f, %.3f) mode=%s armed=%d",
                    taskNameStub(kind), ref_x, ref_y, distance_to_waypoint,
                    ref_x, ref_y, distance_to_waypoint, distance_to_waypoint,
                    static_cast<double>(drop_waypoint_tolerance),
                    0.091f, -0.120f, "OFFBOARD", 1);
  ROS_WARN_THROTTLE(1.0,
                    "%s: no stable detection after %.1fs and %.3f m off the "
                    "authored waypoint (tolerance %.2f m); closing that gap "
                    "to drop on (%.2f, %.2f). Abort at %.1f s from the task "
                    "start if it cannot get there",
                    taskNameStub(kind), static_cast<double>(drop_vision_timeout),
                    distance_to_waypoint,
                    static_cast<double>(drop_waypoint_tolerance), ref_x, ref_y,
                    static_cast<double>(task_timeout));
}

void check_drop_command(TaskKindStub kind, bool vision_confirmed)
{
  const bool hazard = (kind == TASK_HAZARD_STUB);
  ROS_INFO("Drop command sent: %s (servo 1 -> %s) %s",
           hazard ? "DROP_FIRE" : "DROP_RESCUE",
           hazard ? "fire_drop_angle" : "rescue_drop_angle",
           vision_confirmed ? "" : "[no vision confirmation for this drop]");
}

// servo_control/src/servo_controller_node.cpp
void check_servo_angles(double drop_safe_angle, double fire_drop_angle,
                        double rescue_drop_angle, int drop_channel,
                        double drop_action_seconds, double previous_angle,
                        double angle, const std::string &command,
                        const std::string &pending_ack, double held_angle)
{
  ROS_WARN("A drop servo angle is outside 0..180 (safe %.1f, fire %.1f, "
           "rescue %.1f); setAngle clamps, so the servo would not go where "
           "the parameter says",
           drop_safe_angle, fire_drop_angle, rescue_drop_angle);
  ROS_WARN("fire_drop_angle (%.1f) and rescue_drop_angle (%.1f) are only %.1f "
           "deg apart, so the mechanism cannot tell the two materials apart",
           fire_drop_angle, rescue_drop_angle,
           std::fabs(fire_drop_angle - rescue_drop_angle));
  ROS_WARN("drop_safe_angle (%.1f) is within 5 deg of a drop angle "
           "(fire %.1f, rescue %.1f); the parked position would already be a "
           "release position",
           drop_safe_angle, fire_drop_angle, rescue_drop_angle);
  ROS_INFO("Drop servo: channel %d parked at %.1f deg; DROP_FIRE -> %.1f deg, "
           "DROP_RESCUE -> %.1f deg, hold %.2f s",
           drop_channel, drop_safe_angle, fire_drop_angle, rescue_drop_angle,
           drop_action_seconds);
  ROS_INFO("%s: drop servo %d -> %.1f deg (was %.1f), releasing, ack in %.2f s",
           command.c_str(), drop_channel, angle, previous_angle,
           drop_action_seconds);
  ROS_INFO("%s complete; drop servo held at %.1f deg", pending_ack.c_str(),
           held_angle);
}

// ---------------------------------------------------------------------------
// Added 2026-09-30 (bench angle entry) - the four strings servo_controller_node
// gained with /servo_debug_cmd.  The two banner lines carry NO variadic
// arguments at all, which is exactly why they are here: -Wformat=2 also flags a
// literal that contains a stray '%' while the call passes nothing to fill it,
// and that is precisely the mistake a hand-written help text invites.  The ON
// banner is also the only place in this file where the source escapes quotes
// inside a format string, so the escaping gets compiled at least once.
// ---------------------------------------------------------------------------
void check_servo_debug(const std::string &text)
{
  ROS_WARN("angle_debug is ON: /servo_debug_cmd accepts "
           "\"SET_DROP <deg>\", \"SET_CAMERA <deg>\", \"PARK\", \"STATUS\"; "
           "replies come back on /servo_debug_ack. This is a bench setting - "
           "it must never be on for a competition run.");
  ROS_INFO("angle_debug is off; /servo_debug_cmd is not subscribed, so only "
           "the four flight commands can move a servo");
  ROS_ERROR_THROTTLE(
      1.0, "Ignoring servo debug command before successful PCA9685 initialization");
  ROS_INFO("servo debug reply: %s", text.c_str());
  ROS_ERROR("Ignoring non-finite servo angle");
}

// The %zu arguments are std::vector<...>::size() and must stay size_t; the
// startup manifest lines carry %s/double only.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Added 2026-09-30 while fixing "the planned path is not visible in Foxglove".
// The %zu arguments are std::vector<...>::size() and must stay size_t; the
// startup manifest lines carry %s/double only.
// ---------------------------------------------------------------------------
void check_visualisation_manifest(double output_traj_height, const char *frame,
                                  size_t global_pts, size_t local_pts,
                                  size_t a_star_paths, double z)
{
  if (output_traj_height >= 0.0)
    ROS_INFO("=== Visualisation (all latched, frame '%s', z = %.2f m) ===",
             frame, output_traj_height);
  else
    ROS_INFO("=== Visualisation (all latched, frame '%s', z = current pose z) ===",
             frame);
  ROS_INFO("  /visual_global_path      nav_msgs/Path              reference path");
  ROS_INFO("  Set the 3D panel's fixed frame to '%s' to see them without a TF chain.",
           frame);
  ROS_INFO("Planning disabled: replanning stopped, last plan left on screen "
           "(frozen). The follower holds until /trigger_plan goes true.");
  ROS_INFO_THROTTLE(5.0,
                    "EGO visualisation: global_path=%zu pts local_traj=%zu pts "
                    "a_star=%zu paths  frame=%s  z=%.2f",
                    global_pts, local_pts, a_star_paths, frame, z);
}

// ---------------------------------------------------------------------------
// Added 2026-10-07 with field_yaw_deg, the +x / -y take-off heading switch.
// applyRouteFrameYaw() in rescue_mission.cpp is the one place where a wrong
// field_yaw_deg becomes visible BEFORE the aircraft moves, so its four log
// calls are mirrored here verbatim (only geometry_msgs::Point -> FieldPoint
// and fieldToOdom -> the stub below).  Two details are load bearing and are the
// reason this function is worth carrying: the route name is a const char * and
// must stay %s (passing a std::string to a varargs format is undefined, and it
// happens to link because the first member is the pointer), and route.size() is
// a size_t that must stay %zu (-%d would read four bytes of an eight byte value
// on this toolchain and print a plausible but wrong count).
// ---------------------------------------------------------------------------
struct FieldPoint
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

static FieldPoint fieldToOdomStub(double yaw_rad, const FieldPoint &field)
{
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  FieldPoint out;
  out.x = c * field.x + s * field.y;
  out.y = -s * field.x + c * field.y;
  out.z = field.z;
  return out;
}

void check_route_frame(float field_yaw_deg,
                       const std::vector<FieldPoint> &hazard_route,
                       const std::vector<FieldPoint> &person_route,
                       const std::vector<FieldPoint> &return_route,
                       bool have_rescue_drop_point,
                       const FieldPoint &rescue_drop_point,
                       float safe_field_min_x, float safe_field_max_x,
                       float safe_field_min_y, float safe_field_max_y)
{
  const double yaw_rad =
      static_cast<double>(field_yaw_deg) * 3.14159265358979323846 / 180.0;
  if (std::fabs(field_yaw_deg) < 1e-3f)
  {
    ROS_INFO("Route frame: field_yaw_deg=0, so odom and the field frame coincide "
             "here and the route tables are used exactly as authored.");
    return;
  }

  // Copies, because the real call rotates the members in place and is then done
  // with the field-frame values forever.
  std::vector<FieldPoint> hazard = hazard_route;
  std::vector<FieldPoint> person = person_route;
  std::vector<FieldPoint> back = return_route;

  auto rotate_route = [&yaw_rad](const char *route_name,
                                 std::vector<FieldPoint> &route) {
    if (route.empty())
    {
      return;
    }
    const FieldPoint first = route.front();
    const FieldPoint last = route.back();
    for (std::size_t i = 0; i < route.size(); ++i)
    {
      route[i] = fieldToOdomStub(yaw_rad, route[i]);
    }
    ROS_INFO("  %s: %zu points; first (%.2f, %.2f) -> (%.2f, %.2f), "
             "last (%.2f, %.2f) -> (%.2f, %.2f)",
             route_name, route.size(), first.x, first.y, route.front().x,
             route.front().y, last.x, last.y, route.back().x, route.back().y);
  };

  ROS_WARN("Route frame: the airframe's heading in the field frame is %.1f deg, "
           "so every field-frame route point is published rotated by %.1f deg "
           "into odom. The numbers in the launch file still describe the FIELD.",
           static_cast<double>(field_yaw_deg),
           -static_cast<double>(field_yaw_deg));
  rotate_route("hazard_route", hazard);
  rotate_route("person_route", person);
  rotate_route("return_route", back);

  if (have_rescue_drop_point)
  {
    const FieldPoint drawn = rescue_drop_point;
    const FieldPoint moved = fieldToOdomStub(yaw_rad, rescue_drop_point);
    ROS_INFO("  drawn rescue drop (%.2f, %.2f) -> (%.2f, %.2f) in odom",
             drawn.x, drawn.y, moved.x, moved.y);
  }

  // The one number that can be compared against the map by eye.  Necessary
  // because the measured odom yaw is 0 at boot by construction, so reading the
  // yaw back cannot separate a right field_yaw_deg from a wrong one; the pad
  // sits at a corner of the field rather than at its centre, so no two of the
  // four quarter turns put the box in the same place -- a box centred on the
  // origin would be rotation-invariant and tell you nothing.
  double min_x = 1.0e9;
  double max_x = -1.0e9;
  double min_y = 1.0e9;
  double max_y = -1.0e9;
  const float xs[2] = {safe_field_min_x, safe_field_max_x};
  const float ys[2] = {safe_field_min_y, safe_field_max_y};
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 2; ++j)
    {
      FieldPoint corner;
      corner.x = xs[i];
      corner.y = ys[j];
      corner.z = 0.0;
      const FieldPoint rotated = fieldToOdomStub(yaw_rad, corner);
      min_x = std::min(min_x, rotated.x);
      max_x = std::max(max_x, rotated.x);
      min_y = std::min(min_y, rotated.y);
      max_y = std::max(max_y, rotated.y);
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

// ---------------------------------------------------------------------------
// Added 2026-10-17, after the second real-aircraft crash ("it crashed before
// even reaching the task point, using ssd").
//
// The theme of this block is the same mistake in two places: a deadline that is
// expressed as a WALL TIME while the thing it waits for is expressed as a
// MESSAGE ARRIVAL. Five of these six strings carry a duration, and every one of
// them is a double coming out of ros::Time arithmetic - never a ros::Duration
// and never a float member promoted implicitly, because varargs promotes float
// to double silently and a "%f" on a float member is the classic way to print
// 0.000 in a log and spend an afternoon on it.
//
// Three of them are also worth compiling for a second reason: they are the only
// strings in the project that mention a parameter name (require_vision_ready,
// position_tolerance) inline, so a literal '%' typed into that prose - the kind
// of thing "-Wformat=2" catches and nothing else does - would ship silently.
// ---------------------------------------------------------------------------
void check_prearm_gate(bool require_servo_ready_, bool require_vision_ready_,
                       float vision_ready_timeout_,
                       float route_no_motion_timeout_, double alive_at,
                       double now, bool vision_wait_reported_, double waited)
{
  // Two %s fed by ternaries, then two floats widened with static_cast<double>
  // exactly as the source does. %.0f on a float member would be UB in varargs
  // on any ABI that passes float in a narrower register.
  ROS_INFO("Pre-arm gate: servo=%s vision=%s (vision needs a detection message "
           "within 2.0 s of arming, timeout %.0fs) route_no_motion=%.1fs",
           require_servo_ready_ ? "required" : "not required",
           require_vision_ready_ ? "required" : "NOT required",
           static_cast<double>(vision_ready_timeout_),
           static_cast<double>(route_no_motion_timeout_));

  if (waited > static_cast<double>(vision_ready_timeout_))
  {
    if (!vision_wait_reported_)
    {
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
    return;
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
}

// The manual-start counterpart. Note that it takes NO format argument in one of
// its two branches: a string with no conversions at all is legal, and is here so
// that -Wformat=2 also vets the apostrophe-free prose for a stray '%'.
void check_manual_arm_warning(bool require_vision_ready_, double alive_at,
                              double now)
{
  if (!require_vision_ready_)
  {
    return;
  }
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

// The two route-progress lines. %zu does not appear here, but the retry line is
// the one that printed a plausible-looking lie in the 2026-10-17 log ("stalled
// ... resending it") with no distance attached, which is why the distance is
// now the first argument and position_tolerance is spelled out as a float.
void check_route_progress(double current_x, double current_y, double target_x,
                          double target_y, double route_goal_started_at_,
                          double now, float position_tolerance_,
                          int navigation_retries_, int max_navigation_retries_)
{
  const double to_goal =
      std::hypot(current_x - target_x, current_y - target_y);

  ROS_ERROR("Waypoint %.2f, %.2f never moved the vehicle: %.1f s since the "
            "goal was published, %.3f m still to go (position_tolerance "
            "%.2f), and no moving /cmd_vel ever arrived. The planner is "
            "refusing this goal - read EGO's 'drone is in obstacle' / "
            "window_end_inflated / goal_inflated lines before the next leg.",
            target_x, target_y, now - route_goal_started_at_, to_goal,
            static_cast<double>(position_tolerance_));

  ROS_WARN("Planner stalled %.1f m from waypoint (%.2f, %.2f); resending it, "
           "retry %d/%d. If this repeats at the same distance, the goal or "
           "the vehicle is inside EGO's inflated map and the retry is "
           "pointless - check EGO's goal_inflated / 'drone is in obstacle'.",
           to_goal, target_x, target_y, navigation_retries_,
           max_navigation_retries_);
}

// ego_planner_2d_node.cpp, the braking branch that runs BEFORE discretisation.
// The one detail that matters here is %zu on a size_t that was copied out of
// planned_traj_.size() while the mutex was held - it must not be narrowed to
// int for a "%d", because this is exactly the count that has to be trusted when
// reading a post-mortem ("dropped a 3-point trajectory").
void check_ego_start_brake(double local_x, double local_y, size_t dropped)
{
  ROS_WARN_THROTTLE(
      1.0,
      "EGO braking: the vehicle's own cell (%.2f, %.2f) is inside the "
      "inflated obstacle band, so no plan from here can be safe. Dropped a "
      "%zu-point trajectory instead of flying it to its end; the mission "
      "will hold. Move the waypoint or the obstacle - this is geometry, not "
      "tuning.",
      local_x, local_y, dropped);
}

// gc_2026_sim/src/px4_sensor_rig.cpp, the simulated payload release.
// The types are what matter here: model_name is a char[48] built by snprintf and
// passed straight into %s (no std::string temporary to dangle), buffer_size is a
// cast from sizeof, and x/y/z/depth are doubles that must not be handed to %d.
void check_payload_release(const std::string &command,
                           const std::string &service,
                           const std::string &kind, const char *model_name,
                           const char *status_message, double x, double y,
                           double z, double depth, int buffer_size,
                           bool payload_on_drop, bool spawn_available,
                           bool spawn_ok, bool sdf_fits)
{
  if (!payload_on_drop)
  {
    ROS_WARN("GPIO %s acked, but payload_on_drop is false: the mission will "
             "believe a payload was released and nothing will appear in "
             "Gazebo", command.c_str());
    return;
  }
  ROS_INFO("GPIO %s acked; a '%s' payload will be released from the next "
           "ground-truth pose", command.c_str(), kind.c_str());
  if (!spawn_available)
  {
    ROS_WARN("Service %s is not available, so the %s payload cannot be "
             "spawned; the drop is still real as far as the mission is "
             "concerned", service.c_str(), kind.c_str());
    return;
  }
  if (!sdf_fits)
  {
    ROS_ERROR("Payload SDF for '%s' did not fit in %d bytes; spawn skipped",
              model_name, buffer_size);
    return;
  }
  if (!spawn_ok)
  {
    ROS_WARN("Call to %s failed for payload '%s'", service.c_str(), model_name);
    ROS_WARN("Could not spawn payload '%s': %s", model_name, status_message);
    return;
  }
  ROS_INFO("Payload '%s' released at world (%.2f, %.2f, %.2f), %.2f m below "
           "the body; gravity takes it from here",
           model_name, x, y, z, depth);
}

// rescue_mission.cpp, the TAKEOFF branch of controlCallback (2026-10-18).
// Mirrored because it is the only record of what the aircraft is being told
// during the climb, and because it is easy to get the types wrong here: every
// argument is a double (geometry_msgs::Point members are float64, max_velocity_
// is a float that must be widened before varargs, and std::hypot returns a
// double), so every specifier has to stay %f - not one %d.
void check_takeoff_setpoint(double home_x, double home_y, double cmd_z,
                           float max_velocity_, double current_x,
                           double current_y, double current_z)
{
  ROS_INFO_THROTTLE(0.5,
                    "TAKEOFF setpoint: pos=(%.2f, %.2f, %.2f) climb=%.2f m/s "
                    "odom=(%.2f, %.2f, %.2f) d=%.3f",
                    home_x, home_y, cmd_z,
                    static_cast<double>(max_velocity_),
                    current_x, current_y, current_z,
                    std::hypot(current_x - home_x, current_y - home_y));
}

// rescue_mission.cpp, the TAKEOFF branch: the stall diagnostic added 2026-10-18.
// TAKEOFF is the only state with no timeout, and its exit is a position
// predicate, so when the vehicle cannot reach home + fly_height this line is
// the only thing that says so - which is exactly why it is mirrored here.
// Every argument is a double except the three integers-as-%s/%d at the end:
// the tolerances are floats and must be widened before varargs, so no %d may
// appear among them.
void check_takeoff_waiting(double state_age, double target_x, double target_y,
                           double target_z, double odom_x, double odom_y,
                           double odom_z, double res_x, double res_y,
                           double res_z, float position_tolerance_,
                           float altitude_tolerance_, const char *dwell,
                           int requested, int ack, const char *mode)
{
  ROS_WARN_THROTTLE(2.0,
                    "TAKEOFF waiting %.1f s: target=(%.2f, %.2f, %.2f) "
                    "odom=(%.2f, %.2f, %.2f) residual=(%.2f, %.2f, %.2f) "
                    "tol=(%.2f, %.2f) dwell=%s "
                    "camera_down(requested=%d ack=%d) mode=%s",
                    state_age,
                    target_x, target_y, target_z,
                    odom_x, odom_y, odom_z,
                    res_x, res_y, res_z,
                    static_cast<double>(position_tolerance_),
                    static_cast<double>(altitude_tolerance_),
                    dwell,
                    requested,
                    ack,
                    mode);
}
