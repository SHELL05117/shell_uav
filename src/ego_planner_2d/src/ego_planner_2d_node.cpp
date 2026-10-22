/*
 * @Function: 2D Ego-Planner local planner node for ROS1 (melodic / noetic)
 * @Ported from: JackJu-HIT/Ego-Planner-2D-ROS2 (MIT License, develop branch)
 * @Original algorithm: ZJU-FAST-Lab ego-planner
 *
 * The node receives the vehicle pose (odometry), a reference (global) path and
 * obstacle points, then replans a smooth, collision-free 2D local trajectory
 * with the Ego-Planner (B-spline optimization + A* rebound).
 */
#include <ego_planner_2d/ego_planner_2d_node.h>

#include <algorithm>

EgoPlanner2DNode::EgoPlanner2DNode()
    : nh_(), pnh_("~")
{
  // ---------- parameters ----------
  pnh_.param("max_vel", max_vel_, 2.0);
  pnh_.param("max_acc", max_acc_, 3.0);
  pnh_.param("max_jerk", max_jerk_, 4.0);
  pnh_.param("map_resolution", map_resolution_, 0.1);
  pnh_.param("map_x_size", map_x_size_, 50.0);
  pnh_.param("map_y_size", map_y_size_, 50.0);
  pnh_.param("map_inflate_value", map_inflate_value_, 0.5);
  pnh_.param("sensing_range", sensing_range_, 10.0);
  pnh_.param("planning_rate", planning_rate_, 5.0);
  pnh_.param("path_sample_interval", path_sample_interval_, 0.3);
  pnh_.param("obstacle_height_band", obstacle_height_band_, 1.0);
  pnh_.param("local_window_points", local_window_points_, 50);
  pnh_.param("min_traj_points", min_traj_points_, 10);
  // Hard floor on the number of reference points handed to the optimizer.
  // BsplineOptimizer needs > 2 * order (= 6) control points, i.e. > 4
  // reference points; below that lbfgs is called with n <= 0 and returns
  // LBFGSERR_INVALID_N (-1021). 8 leaves real headroom for the optimizer.
  pnh_.param("min_ref_points", min_ref_points_, 8);
  pnh_.param("obstacle_accumulate", obstacle_accumulate_, true);
  pnh_.param("global_frame", global_frame_, std::string("map"));

  std::string pose_source = "odom";
  pnh_.param("pose_source", pose_source, std::string("odom"));
  pose_source_tf_ = (pose_source == "tf");
  pnh_.param("base_frame", base_frame_, std::string("base_link"));
  pnh_.param("output_traj_height", output_traj_height_, -1.0);
  if (pose_source_tf_)
    ROS_INFO("Pose source: TF (%s -> %s), odom topic is ignored.",
             global_frame_.c_str(), base_frame_.c_str());

  cur_pose_.x = 0.0;
  cur_pose_.y = 0.0;
  cur_pose_.z = 0.0; // yaw
  cur_pose_.v = 0.0;

  // ---------- publishers ----------
  // LATCHED on every visualisation topic, not just the global path.
  //
  // These topics are the only way to see what the planner is doing, and they are
  // published from a 5 Hz timer - so a panel that subscribes late (the operator
  // adds the topic in Foxglove after the flight has started, or reconnects) had
  // to wait for the next tick and got nothing at all once planning stopped.
  // With a latch the last message is retained and delivered to any new
  // subscriber immediately, whatever the planner is doing at that moment.
  //
  // Latching is free here: the queue is 1 and the payload is a few kB.
  global_path_pub_ = nh_.advertise<nav_msgs::Path>("visual_global_path", 1, true);
  a_star_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectories", 1, true);
  local_traj_pub_ = nh_.advertise<nav_msgs::Path>("visual_local_trajectory", 1, true);
  obs_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("visual_obstacles", 1);
  obs_local_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("visual_local_obstacles", 1);

  // ---------- subscribers (all remappable) ----------
  tf_buffer_.reset(new tf2_ros::Buffer);
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  odom_sub_ = nh_.subscribe("odom", 10, &EgoPlanner2DNode::odomCallback, this);
  global_path_sub_ = nh_.subscribe("global_path", 10, &EgoPlanner2DNode::globalPathCallback, this);
  goal_sub_ = nh_.subscribe("goal", 10, &EgoPlanner2DNode::goalCallback, this);
  clicked_point_sub_ = nh_.subscribe("clicked_point", 10, &EgoPlanner2DNode::clickedPointCallback, this);
  initial_pose_sub_ = nh_.subscribe("initialpose", 10, &EgoPlanner2DNode::initialPoseCallback, this);
  obstacles_sub_ = nh_.subscribe("obstacles", 10, &EgoPlanner2DNode::obstaclesCallback, this);
  scan_sub_ = nh_.subscribe("scan", 10, &EgoPlanner2DNode::scanCallback, this);
  trigger_sub_ = nh_.subscribe("trigger_plan", 10, &EgoPlanner2DNode::triggerCallback, this);

  // ---------- planner core ----------
  ego_planner_ = std::make_shared<PlannerInterface>();
  ego_planner_->initParam(max_vel_, max_acc_, max_jerk_);
  ego_planner_->initEsdfMap(map_x_size_, map_y_size_, map_resolution_, map_inflate_value_);

  // ---------- planning loop ----------
  timer_ = nh_.createTimer(ros::Duration(1.0 / planning_rate_),
                           &EgoPlanner2DNode::planningLoop, this);

  ROS_INFO("ego_planner_2d node ready (rate %.1f Hz).", planning_rate_);
  ROS_INFO("Subscribed: odom / global_path / goal / clicked_point / initialpose / obstacles / scan / trigger_plan");
  ROS_INFO("=== How to use (RViz) ===");
  ROS_INFO("1. 2D Pose Estimate  -> reset vehicle pose (when no odometry)");
  ROS_INFO("2. Publish Point     -> drop an obstacle for testing");
  ROS_INFO("3. 2D Nav Goal       -> set a goal, a straight reference path is generated");
  ROS_INFO("   (or publish a nav_msgs/Path on 'global_path' from your global planner)");
  ROS_INFO("4. rostopic pub /trigger_plan std_msgs/Bool \"data: true|false\" to start/stop planning");

  // Printed at startup so the exact topic names, types and heights can be copied
  // into a Foxglove panel without guessing, and so "I cannot see the plan" is
  // checkable against a list instead of against memory. Every one of these is
  // stamped in global_frame_ and drawn at the same z, so a 3D panel whose fixed
  // frame is global_frame_ shows the whole plan as one picture. A panel set to a
  // different fixed frame needs a global_frame_ -> <fixed frame> TF, and when
  // that transform is missing the message is silently dropped by the panel, not
  // by this node.
  if (output_traj_height_ >= 0.0)
    ROS_INFO("=== Visualisation (all latched, frame '%s', z = %.2f m) ===",
             global_frame_.c_str(), output_traj_height_);
  else
    ROS_INFO("=== Visualisation (all latched, frame '%s', z = current pose z) ===",
             global_frame_.c_str());
  ROS_INFO("  /visual_global_path      nav_msgs/Path              reference path");
  ROS_INFO("  /visual_local_trajectory nav_msgs/Path              B-spline / A* trajectory");
  ROS_INFO("  /trajectories            visualization_msgs/MarkerArray  A* rebound paths");
  ROS_INFO("  /visual_obstacles        sensor_msgs/PointCloud2    raw obstacle points");
  ROS_INFO("  /visual_local_obstacles  sensor_msgs/PointCloud2    local obstacle points");
  ROS_INFO("  Set the 3D panel's fixed frame to '%s' to see them without a TF chain.",
           global_frame_.c_str());
}

// ---------------------------------------------------------------------------
// callbacks
// ---------------------------------------------------------------------------
void EgoPlanner2DNode::updatePoseFromTf()
{
  geometry_msgs::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform(global_frame_, base_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& e)
  {
    ROS_WARN_THROTTLE(2.0, "Cannot look up pose TF (%s -> %s): %s",
                      global_frame_.c_str(), base_frame_.c_str(), e.what());
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  cur_pose_.x = tf.transform.translation.x;
  cur_pose_.y = tf.transform.translation.y;
  cur_z_ = tf.transform.translation.z;
  tf2::Quaternion q(tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w);
  cur_pose_.z = tf2::getYaw(q); // PathPoint.z is reused to store yaw
  has_pose_ = true;
}

void EgoPlanner2DNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (pose_source_tf_)
    return; // odom topic is ignored when pose comes from TF

  std::lock_guard<std::mutex> lock(data_mutex_);
  cur_pose_.x = msg->pose.pose.position.x;
  cur_pose_.y = msg->pose.pose.position.y;
  cur_z_ = msg->pose.pose.position.z;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
  cur_pose_.z = tf2::getYaw(q); // PathPoint.z is reused to store yaw
  has_pose_ = true;
}

void EgoPlanner2DNode::globalPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  if (!msg->header.frame_id.empty() && msg->header.frame_id != global_frame_)
  {
    ROS_WARN("Ignoring global path in frame '%s'; expected '%s'.",
             msg->header.frame_id.c_str(), global_frame_.c_str());
    clearPlan();
    return;
  }
  if (msg->poses.empty())
  {
    ROS_WARN("Received empty global path, ignored.");
    clearPlan();
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  planned_traj_.clear();
  global_plan_traj_.clear();
  for (const auto& pose_stamped : msg->poses)
  {
    PathPoint p;
    p.x = pose_stamped.pose.position.x;
    p.y = pose_stamped.pose.position.y;
    p.z = 0.0;
    p.v = 0.0;
    global_plan_traj_.push_back(p);
  }
  has_valid_global_path_ = true;
  needs_replan_ = true;
  ROS_INFO("Received global path with %zu points.", global_plan_traj_.size());
}

void EgoPlanner2DNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (!msg->header.frame_id.empty() && msg->header.frame_id != global_frame_)
  {
    ROS_WARN("Ignoring goal in frame '%s'; expected '%s'.",
             msg->header.frame_id.c_str(), global_frame_.c_str());
    clearPlan();
    return;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  planned_traj_.clear();

  PathPoint goal;
  goal.x = msg->pose.position.x;
  goal.y = msg->pose.position.y;
  goal.z = 0.0;
  goal.v = 0.0;

  PathPoint start;
  if (has_pose_)
  {
    start = cur_pose_;
  }
  else
  {
    ROS_WARN("Goal received before any pose, using (0,0) as start.");
  }

  global_plan_traj_.clear();
  generateStraightPath(start, goal, global_plan_traj_);
  has_valid_global_path_ = true;
  needs_replan_ = true;
  ROS_INFO("New goal (%.2f, %.2f), straight reference path with %zu points generated.",
           goal.x, goal.y, global_plan_traj_.size());
}

void EgoPlanner2DNode::clearPlan()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  planned_traj_.clear();
  global_plan_traj_.clear();
  a_star_pathes_.clear();
  has_valid_global_path_ = false;
  needs_replan_ = false;
}

void EgoPlanner2DNode::clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  addObstacleAtPosition(msg->point.x, msg->point.y);
}

void EgoPlanner2DNode::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cur_pose_.x = msg->pose.pose.position.x;
  cur_pose_.y = msg->pose.pose.position.y;
  cur_z_ = msg->pose.pose.position.z;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
  cur_pose_.z = tf2::getYaw(q);
  has_pose_ = true;

  if (has_valid_global_path_)
  {
    needs_replan_ = true;
    ROS_INFO("Pose reset to (%.2f, %.2f), replanning.", cur_pose_.x, cur_pose_.y);
  }
}

void EgoPlanner2DNode::obstaclesCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // NOTE: points are assumed to be expressed in the global (map) frame.
  // Transform them beforehand (e.g. with a tf listener in your perception
  // node) if your sensor publishes in its own frame.
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (!obstacle_accumulate_)
  {
    obstacles_.clear();
    obstacle_cells_.clear();
  }

  try
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (addObstaclePointLocked(*iter_x, *iter_y, *iter_z))
        needs_replan_ = true;
    }
  }
  catch (const std::runtime_error& e)
  {
    ROS_WARN("Obstacle cloud has no x/y/z field: %s", e.what());
    return;
  }
}

void EgoPlanner2DNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Convert a 2D laser scan (e.g. RPLIDAR) into obstacle points.
  // The scan frame is transformed into the global frame with the latest
  // available TF (ros::Time(0)); make sure your localization publishes
  // global_frame -> laser_frame (e.g. map -> odom -> base_link -> laser).
  geometry_msgs::TransformStamped tf;
  bool identity = msg->header.frame_id.empty() || msg->header.frame_id == global_frame_;
  if (!identity)
  {
    try
    {
      tf = tf_buffer_->lookupTransform(global_frame_, msg->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN_THROTTLE(2.0, "Cannot transform laser scan (%s -> %s): %s",
                        msg->header.frame_id.c_str(), global_frame_.c_str(), e.what());
      return;
    }
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (!obstacle_accumulate_)
  {
    obstacles_.clear();
    obstacle_cells_.clear();
  }

  geometry_msgs::PointStamped pin, pout;
  pin.header = msg->header;
  pin.header.stamp = ros::Time::now();

  bool added = false;
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    double r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min ||
        r >= msg->range_max - 1e-3)
      continue;

    double a = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    pin.point.x = r * std::cos(a);
    pin.point.y = r * std::sin(a);
    pin.point.z = 0.0;

    if (identity)
      pout = pin;
    else
      tf2::doTransform(pin, pout, tf);

    if (addObstaclePointLocked(pout.point.x, pout.point.y, pout.point.z))
      added = true;
  }

  if (added || !obstacle_accumulate_)
    needs_replan_ = true;
}

void EgoPlanner2DNode::triggerCallback(const std_msgs::Bool::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  should_plan_ = msg->data;
  if (should_plan_)
  {
    needs_replan_ = true;
    stop_reported_ = false;
  }
  ROS_INFO("Planning %s.", should_plan_ ? "enabled" : "disabled");
}

void EgoPlanner2DNode::addObstacleAtPosition(double x, double y)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (addObstaclePointLocked(x, y, 0.0))
  {
    needs_replan_ = true;
    ROS_INFO("Obstacle added at (%.2f, %.2f), total %zu.", x, y, obstacles_.size());
  }
}

// Filter (height band, sensing range), deduplicate on the map grid and store
// an obstacle point. Returns true when the point was actually added.
// The caller MUST hold data_mutex_.
bool EgoPlanner2DNode::addObstaclePointLocked(double x, double y, double z)
{
  // keep only obstacles close to the current flight height
  if (obstacle_height_band_ > 0.0 &&
      std::fabs(z - cur_z_) > obstacle_height_band_ * 0.5)
    return false;

  // keep only obstacles within sensing range (bounds the local map)
  double dx = x - cur_pose_.x;
  double dy = y - cur_pose_.y;
  if (dx * dx + dy * dy > sensing_range_ * sensing_range_)
    return false;

  // deduplicate on the map grid so a dense cloud does not blow up inflation
  long long ix = static_cast<long long>(std::llround(x / map_resolution_));
  long long iy = static_cast<long long>(std::llround(y / map_resolution_));
  if (!obstacle_cells_.insert(ix * 1000000LL + iy).second)
    return false;

  ObstacleInfo obs;
  obs.x = static_cast<float>(x);
  obs.y = static_cast<float>(y);
  obs.z = static_cast<float>(z);
  obstacles_.push_back(obs);
  return true;
}

// ---------------------------------------------------------------------------
// planning
// ---------------------------------------------------------------------------
void EgoPlanner2DNode::planningLoop(const ros::TimerEvent& /*event*/)
{
  if (pose_source_tf_)
    updatePoseFromTf();

  // The pose is refreshed before this early return on purpose: the mission may
  // switch planning back on at any moment, and a stale pose would make the first
  // replan after that start from the wrong place.
  if (!should_plan_)
  {
    reportPlanningDisabled();
    return;
  }

  PathPoint local_pose;
  std::vector<ObstacleInfo> local_obstacles;
  std::vector<PathPoint> local_global_path;
  bool ready = false;
  bool has_pose = false;
  bool has_global_path = false;
  size_t global_path_points = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    has_pose = has_pose_;
    has_global_path = has_valid_global_path_;
    global_path_points = global_plan_traj_.size();
    ready = has_pose_ && has_valid_global_path_ && global_plan_traj_.size() >= 2;
    if (ready)
    {
      local_pose = cur_pose_;
      local_obstacles = obstacles_;
      local_global_path = global_plan_traj_;
    }
  }
  if (!ready)
  {
    ROS_WARN_THROTTLE(
        2.0, "EGO waiting for inputs: pose=%s global_path=%s points=%zu",
        has_pose ? "ready" : "missing",
        has_global_path ? "ready" : "missing", global_path_points);
    publishAll();
    return;
  }

  // 1. keep only obstacles within sensing range (and prune stale ones)
  {
    std::vector<ObstacleInfo> in_range;
    in_range.reserve(local_obstacles.size());
    for (const auto& obs : local_obstacles)
    {
      double dist = std::hypot(obs.x - local_pose.x, obs.y - local_pose.y);
      if (dist < sensing_range_)
        in_range.push_back(obs);
    }
    local_obstacles.swap(in_range);
  }
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    obstacles_ = local_obstacles;
  }

  // 2. refresh the local map (centered on the current position, then inflate)
  ego_planner_->setGridMap(local_pose);
  ego_planner_->resetMap();
  ego_planner_->setObstacles(local_obstacles);

  // 3. replan on demand / on collision / when the local trajectory is exhausted
  bool need = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    need = needs_replan_;
  }
  if (need || planned_traj_.size() < static_cast<size_t>(min_traj_points_) ||
      collisionDetection(planned_traj_))
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      needs_replan_ = false;
    }
    replan(local_pose, local_global_path, local_obstacles);
  }

  publishAll();
}

void EgoPlanner2DNode::replan(const PathPoint& local_pose,
                              const std::vector<PathPoint>& global_path,
                              const std::vector<ObstacleInfo>& local_obstacles)
{
  // 0. The vehicle's own cell inside the inflated band is the one failure that
  // must NOT fall through to "keep the previous trajectory".
  //
  // The rule below ("keep the last valid trajectory so the vehicle continues
  // along it") exists because dropping a good plan leaves the mission waiting
  // for an arrival that can no longer happen. That reasoning holds when the
  // *end* of the path is unreachable: there is still a known-safe piece of path
  // left to fly. It does not hold when the START is occupied, because then
  // there is no safe piece at all - the far end of the stale plan is what the
  // controller will stop at, and on 2026-10-17 that end was inside a real
  // object 0.3 m from hazard_route[1]. EGO dutifully reported "the drone is in
  // obstacle" for 40 replans while the follower kept flying the stale 3-point
  // trajectory and the mission kept waiting.
  //
  // Publishing an empty path is a brake, not a stall: EgoTrajectoryController
  // treats an empty trajectory as "command zero velocity" (and says so), the
  // mission sees the stale command within planner_velocity_timeout_ and holds
  // its own position, and its route_no_motion_timeout_ names the reason within
  // seconds. It is self-recovering: planned_traj_.size() < min_traj_points_
  // forces a replan every cycle, so the moment the start becomes free again a
  // normal plan replaces the empty one.
  {
    PathPoint start_probe = local_pose;
    if (ego_planner_->getInflateOccupancy(start_probe))
    {
      size_t dropped = 0;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        dropped = planned_traj_.size();
        planned_traj_.clear();
        a_star_pathes_.clear();
      }
      ROS_WARN_THROTTLE(
          1.0,
          "EGO braking: the vehicle's own cell (%.2f, %.2f) is inside the "
          "inflated obstacle band, so no plan from here can be safe. Dropped a "
          "%zu-point trajectory instead of flying it to its end; the mission "
          "will hold. Move the waypoint or the obstacle - this is geometry, not "
          "tuning.",
          local_pose.x, local_pose.y, dropped);
      return;
    }
  }

  // 1. discretize the reference path and restart from the closest point
  std::vector<PathPoint> discretized;
  discretizeTrajectory(global_path, discretized, path_sample_interval_);
  if (discretized.size() < 2)
  {
    // This used to return silently, so a reference path that collapsed to a
    // single point produced "nothing happened" with no line in the log.
    ROS_WARN_THROTTLE(1.0,
                      "EGO replan skipped: reference path discretized to %zu "
                      "point(s) at %.2f m spacing (global path has %zu point(s))",
                      discretized.size(), path_sample_interval_,
                      global_path.size());
    return;
  }

  size_t nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < discretized.size(); ++i)
  {
    double dist = distance(discretized[i], local_pose);
    if (dist < min_dist)
    {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  std::vector<PathPoint> ref_path;
  ref_path.push_back(local_pose);
  for (size_t i = nearest_idx; i < discretized.size(); ++i)
    ref_path.push_back(discretized[i]);

  // 2. re-discretize from the vehicle and truncate to the local window
  discretized.clear();
  discretizeTrajectory(ref_path, discretized, path_sample_interval_);

  size_t length = static_cast<size_t>(local_window_points_);
  if (discretized.size() < length)
    length = discretized.size();
  ref_path.assign(discretized.begin(), discretized.begin() + length);

  // 2b. Guarantee the optimizer gets enough points.
  //
  // The reference path is truncated to local_window_points * path_sample_interval
  // metres ahead of the vehicle. Near the end of a leg the REMAINING path is
  // shorter than that window, so the fixed path_sample_interval yields only a
  // handful of points - at 0.20 m spacing a 0.4 m remainder gives 4 points.
  // UniformBspline::parameterizeToBspline turns K points into K + 2 control
  // points, and BsplineOptimizer::rebound_optimize needs
  // 2 * ((K + 2) - 2 * order_) > 0 with order_ = 3, i.e. K >= 5. With K = 4 the
  // solver was called with 0 variables, returned LBFGSERR_INVALID_N (-1021)
  // and the plan was dropped.
  //
  // Re-sample the SAME span at a tighter interval. The endpoint (and therefore
  // where the controller will stop) does not move; only the sampling density
  // changes, which is what the optimizer's free-variable count depends on.
  if (ref_path.size() < static_cast<size_t>(min_ref_points_))
  {
    double span = 0.0;
    for (size_t i = 0; i + 1 < ref_path.size(); ++i)
      span += distance(ref_path[i], ref_path[i + 1]);

    if (span > 1e-3 && min_ref_points_ > 1)
    {
      const double tighter =
          std::max(span / static_cast<double>(min_ref_points_ - 1) * 0.999, 1e-3);
      std::vector<PathPoint> dense;
      discretizeTrajectory(ref_path, dense, tighter);
      if (dense.size() > ref_path.size())
      {
        ROS_INFO_THROTTLE(
            2.0,
            "EGO re-sampled a short reference path: %zu point(s) over %.3f m at "
            "%.3f m spacing -> %zu point(s) (optimizer needs > 4).",
            ref_path.size(), span, tighter, dense.size());
        ref_path.swap(dense);
      }
    }
  }

  // Where this plan is ALLOWED to end, and whether that endpoint is reachable.
  //
  // This is the diagnostic that the 2026-09-25 ego run needed and did not have.
  // The symptom was "it flew for a while and then stopped" with a completely
  // clean log. The controller stops when the vehicle reaches the last pose of
  // the published trajectory (EgoTrajectoryController measures goal_tolerance
  // against path.poses.back()), and that last pose is NOT the route goal: the
  // reference path is truncated to local_window_points * path_sample_interval
  // metres ahead of the vehicle. So if the optimiser plus A* fallback can only
  // reach partway - the usual reason being that the window end, or the goal
  // itself, lands inside an inflated obstacle - the vehicle flies exactly that
  // far and then idles, with every consumer reporting a healthy system.
  //
  // These two flags are the first thing to read off this line. With
  // map_inflate_value 0.35 the west wall's inner face (x = -0.650) is rasterised
  // to the cell centre at x = -0.61, and 0.35 m of inflation - now measured in
  // metres, see GridMap2D::inflateObstacles - reaches cell centre x = -0.31, so
  // the hazard_route waypoint at (-0.25, -1.80) is 0.06 m outside the band and
  // reachable. Before that fix the test was `dist_grid <= ceil(0.35/0.10)`, i.e.
  // 4 whole cells, the band reached x = -0.21, the waypoint was INSIDE it, and
  // the vehicle could not get closer than x = -0.11 - observed as
  // "NAV cmd: odom=(-0.11, -1.42, ...) goal=(-0.25, -1.80) d=0.406", frozen
  // 0.41 m out against a 0.12 m arrival tolerance.
  const PathPoint window_end = ref_path.back();
  const PathPoint goal_point = global_path.back();
  // getInflateOccupancy takes a non-const PathPoint&, so copy - the same reason
  // collisionDetection() above copies each point into a local.
  PathPoint window_end_probe = window_end;
  PathPoint goal_probe = goal_point;
  const bool window_end_inflated = ego_planner_->getInflateOccupancy(window_end_probe);
  const bool goal_inflated = ego_planner_->getInflateOccupancy(goal_probe);
  const double window_end_to_goal = distance(window_end, goal_point);

  // 3. optimize with the 2D Ego-Planner
  ego_planner_->setPathPoint(ref_path);
  PathPoint pose_copy = local_pose;
  ego_planner_->setCurrentVehiclePos(pose_copy);

  // Plan into locals and only publish the result on success. The previous
  // version cleared planned_traj_ BEFORE calling makePlan(), and
  // publishLocalTrajectory() publishes planned_traj_ unconditionally - so a
  // single failed replan replaced a perfectly good trajectory with an empty
  // nav_msgs::Path. EgoTrajectoryController treats an empty trajectory as
  // "hold position", i.e. the vehicle froze in mid-air while every node
  // reported healthy. That is the reported "ego flew for a while and then
  // stopped": near a waypoint the planner fails, the good plan is destroyed,
  // and the mission keeps waiting for an arrival that can no longer happen.
  std::vector<PathPoint> new_traj;
  std::vector<std::vector<Eigen::Vector2d>> new_a_star_paths;
  ego_planner_->makePlan();
  ego_planner_->getAStarPath(new_a_star_paths);
  ego_planner_->getLocalPlanTrajResults(new_traj);

  if (new_traj.empty())
  {
    ROS_WARN_THROTTLE(1.0,
                      "EGO spline optimization and A* fallback produced no safe "
                      "trajectory: %zu ref point(s), window end (%.2f, %.2f) is "
                      "%.2f m from the goal (%.2f, %.2f); window_end_inflated=%d "
                      "goal_inflated=%d obstacles=%zu a_star_paths=%zu",
                      ref_path.size(), window_end.x, window_end.y,
                      window_end_to_goal, goal_point.x, goal_point.y,
                      window_end_inflated ? 1 : 0, goal_inflated ? 1 : 0,
                      local_obstacles.size(), new_a_star_paths.size());

    // Keep the last valid trajectory so the vehicle continues along it instead
    // of being told to hold position. Report how much of it is left, because a
    // stale trajectory that has already been consumed is the other way this
    // node can strand the vehicle.
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (planned_traj_.empty())
    {
      ROS_WARN_THROTTLE(1.0, "EGO has no previous trajectory to fall back on.");
    }
    else if (collisionDetection(planned_traj_))
    {
      // "Keep the previous plan" is only defensible while that plan is still
      // safe. collisionDetection() is the very predicate that forced this
      // replan (see the caller), so keeping a plan it rejects means the
      // follower keeps tracking a trajectory that is inside the inflated band -
      // the plan the node itself has already called unsafe. That is the exact
      // shape of "it did not compute the path properly and drove into the
      // wall": the replan failed, the stale plan was republished unchanged, and
      // /cmd_vel kept steering along it.
      //
      // Dropping it instead publishes an empty path, which the follower turns
      // into zero velocity (a brake, not a freeze in place - see step 0). The
      // mission then reports route_no_motion_timeout and, with the 2026-10-17
      // ABORT change, lands instead of hovering forever. Recovery is automatic:
      // planned_traj_.size() < min_traj_points_ forces a replan next cycle.
      const size_t dropped = planned_traj_.size();
      planned_traj_.clear();
      a_star_pathes_.clear();
      ROS_WARN_THROTTLE(
          1.0,
          "EGO dropping a %zu-point stale trajectory: it is inside the "
          "inflated band, so keeping it would steer the vehicle into an "
          "obstacle. Braking (empty path) until a plan is found.",
          dropped);
    }
    else
    {
      ROS_WARN_THROTTLE(1.0,
                        "EGO keeping the previous %zu-point trajectory "
                        "(its far end is what the controller will stop at).",
                        planned_traj_.size());
    }
    return;
  }

  size_t planned_size = 0;
  PathPoint planned_end;
  size_t inflated_pts = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    planned_traj_.swap(new_traj);
    a_star_pathes_.swap(new_a_star_paths);
    planned_size = planned_traj_.size();
    if (planned_size > 0)
      planned_end = planned_traj_.back();
    // Audit of what is about to be published. The optimizer now checks the
    // whole spline and A* no longer cuts corners, so this should be 0 for
    // ever; it is printed rather than asserted so a regression shows up as a
    // number in the log instead of as a crash.
    for (const auto &pt : planned_traj_)
    {
      PathPoint p = pt;
      if (ego_planner_->getInflateOccupancy(p))
        ++inflated_pts;
    }
  }
  if (inflated_pts > 0)
  {
    ROS_WARN_THROTTLE(1.0,
                      "EGO published a plan with %zu of %zu point(s) inside "
                      "the inflated band - the planner's clearance check has a "
                      "hole (see bspline_optimizer.cpp / dyn_a_star.cpp).",
                      inflated_pts, planned_size);
  }

  // The number that predicts how far the vehicle will actually go before the
  // controller decides it has arrived: how far the planned end is from the
  // goal. Near 0 means a complete plan; a large value means this leg will
  // stall partway, and the two inflated flags above say why.
  ROS_INFO_THROTTLE(2.0,
                    "EGO planned %zu point(s), end (%.2f, %.2f) is %.3f m "
                    "from the goal (%.2f, %.2f); window_end_inflated=%d "
                    "goal_inflated=%d inflated_pts=%zu obstacles=%zu",
                    planned_size, planned_end.x, planned_end.y,
                    distance(planned_end, goal_point),
                    goal_point.x, goal_point.y,
                    window_end_inflated ? 1 : 0, goal_inflated ? 1 : 0,
                    inflated_pts, local_obstacles.size());
}

bool EgoPlanner2DNode::collisionDetection(const std::vector<PathPoint>& traj)
{
  for (const auto& pt : traj)
  {
    PathPoint p = pt;
    if (ego_planner_->getInflateOccupancy(p))
      return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// publish
// ---------------------------------------------------------------------------
void EgoPlanner2DNode::publishAll()
{
  // One line every 5 s that says how much there is to see on each channel, so
  // "the plan is not visible in Foxglove" can be split into "the node is not
  // publishing it" (this counts a zero) and "the panel is not drawing it" (the
  // counts are healthy). Every drift investigation before this had to guess
  // which of the two it was.
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const double z = (output_traj_height_ >= 0.0) ? output_traj_height_ : cur_z_;
    ROS_INFO_THROTTLE(5.0,
                      "EGO visualisation: global_path=%zu pts local_traj=%zu pts "
                      "a_star=%zu paths  frame=%s  z=%.2f",
                      global_plan_traj_.size(), planned_traj_.size(),
                      a_star_pathes_.size(), global_frame_.c_str(), z);
  }
  publishGlobalPath();
  publishLocalTrajectory();
  publishObstacles();
  publishLocalObstacles();
  publishAStarPaths();
}

void EgoPlanner2DNode::reportPlanningDisabled()
{
  // /trigger_plan is false: the mission has no route to fly, so stop replanning.
  // The visualisation is deliberately NOT cleared.
  //
  // The first version of this did clear it - one empty trajectory plus a marker
  // DELETEALL - and that was wrong for exactly the reason the operator then hit:
  // it made "the planner is idle" indistinguishable from "the planner is
  // broken" in Foxglove, and it erased the answer to "where was it last trying
  // to go" the moment a leg or the whole flight ended. Publishing nothing keeps
  // the last nav_msgs/Path and MarkerArray on screen (Foxglove holds the last
  // message it received), and because those topics are latched, a panel that
  // subscribes afterwards gets it too.
  //
  // Leaving a stale plan visible is safe: nothing consumes it. EgoTrajectory
  // Controller has its own /trigger_plan subscription and commands zero velocity
  // while planning is off, and even a follower without that subscription would
  // see the trajectory go stale after command_timeout_ (0.6 s) and command the
  // same zero.
  if (stop_reported_)
  {
    return;
  }
  stop_reported_ = true;
  ROS_INFO("Planning disabled: replanning stopped, last plan left on screen "
           "(frozen). The follower holds until /trigger_plan goes true.");
}

void EgoPlanner2DNode::publishGlobalPath()
{
  nav_msgs::Path visual_path;
  visual_path.header.stamp = ros::Time::now();
  visual_path.header.frame_id = global_frame_;

  std::lock_guard<std::mutex> lock(data_mutex_);
  // Draw the reference path at the SAME height as the local trajectory.
  //
  // It used to go out at z = 0, i.e. flat in the ground plane, while
  // publishLocalTrajectory() put the trajectory at output_traj_height_ (1.5 m in
  // this mission). That is the "the ego trajectories are not in one plane" the
  // operator noticed first, and in a 3D panel it is worse than cosmetic: a path
  // lying in the floor plane is hidden by the floor grid and by the enclosure
  // walls whenever the view is close to horizontal, which is the default camera
  // angle. One height for the whole plan makes it readable as a single picture.
  const double z = (output_traj_height_ >= 0.0) ? output_traj_height_ : cur_z_;
  for (const auto& path_point : global_plan_traj_)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = visual_path.header;
    pose.pose.position.x = path_point.x;
    pose.pose.position.y = path_point.y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = 1.0;
    visual_path.poses.push_back(pose);
  }
  global_path_pub_.publish(visual_path);
}

void EgoPlanner2DNode::publishLocalTrajectory()
{
  nav_msgs::Path visual_traj;
  visual_traj.header.stamp = ros::Time::now();
  visual_traj.header.frame_id = global_frame_;

  std::vector<PathPoint> traj;
  double z;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    traj = planned_traj_;
    // keep the flight height in the published trajectory; with a 2D-only
    // localization backend (e.g. cartographer) the pose z is always 0, in
    // that case set output_traj_height to the commanded flight height
    z = (output_traj_height_ >= 0.0) ? output_traj_height_ : cur_z_;
  }
  for (const auto& pt : traj)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = visual_traj.header;
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = 1.0;
    visual_traj.poses.push_back(pose);
  }
  local_traj_pub_.publish(visual_traj);
}

void EgoPlanner2DNode::publishObstacles()
{
  std::vector<ObstacleInfo> obstacles;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    obstacles = obstacles_;
  }
  if (obstacles.empty())
    return;

  sensor_msgs::PointCloud2 visual_obs;
  visual_obs.header.stamp = ros::Time::now();
  visual_obs.header.frame_id = global_frame_;
  visual_obs.height = 1;
  visual_obs.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(visual_obs);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(obstacles.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");
  for (const auto& obs : obstacles)
  {
    *iter_x = obs.x;
    *iter_y = obs.y;
    *iter_z = 0.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  obs_pub_.publish(visual_obs);
}

void EgoPlanner2DNode::publishLocalObstacles()
{
  std::vector<ObstacleInfo> obstacles;
  ego_planner_->getObstacles(obstacles);
  if (obstacles.empty())
    return;

  sensor_msgs::PointCloud2 visual_obs;
  visual_obs.header.stamp = ros::Time::now();
  visual_obs.header.frame_id = global_frame_;
  visual_obs.height = 1;
  visual_obs.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(visual_obs);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(obstacles.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");
  for (const auto& obs : obstacles)
  {
    *iter_x = obs.x;
    *iter_y = obs.y;
    *iter_z = 0.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  obs_local_pub_.publish(visual_obs);
}

void EgoPlanner2DNode::publishAStarPaths()
{
  std::vector<std::vector<Eigen::Vector2d>> paths;
  double z = 0.0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    paths = a_star_pathes_;
    // Same height as the other two channels - see the note in publishGlobalPath.
    z = (output_traj_height_ >= 0.0) ? output_traj_height_ : cur_z_;
  }

  visualization_msgs::MarkerArray marker_array;

  // Clear the namespace first, then re-add. A 3D panel keeps every marker until
  // it is deleted or expires, and it keys them by (namespace, id) - so if one
  // frame produced three A* rebound paths and the next produced one, ids 1 and 2
  // would stay on screen forever and the display would show a path the planner is
  // no longer considering. Markers inside one MarkerArray are applied in order,
  // so the DELETEALL that goes out at the front clears the previous frame before
  // the ADDs below redraw the current one. Publishing the array even when there
  // are no paths is what makes the removal happen at all; the old early return
  // left the last non-empty set frozen on screen.
  //
  // orientation.w is set even here: a DELETEALL ignores the pose, but a consumer
  // that normalises the quaternion before it looks at action would see the
  // all-zero value as NaN and drop the marker (see the note on the ADD markers).
  {
    visualization_msgs::Marker clear;
    clear.header.frame_id = global_frame_;
    clear.header.stamp = ros::Time::now();
    clear.ns = "a_star_paths";
    clear.id = 0;
    clear.action = visualization_msgs::Marker::DELETEALL;
    clear.pose.orientation.w = 1.0;
    marker_array.markers.push_back(clear);
  }

  for (size_t i = 0; i < paths.size(); ++i)
  {
    const std::vector<Eigen::Vector2d>& path = paths[i];

    visualization_msgs::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "a_star_paths";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose was never assigned, so it went out as an ALL-ZERO quaternion.
    // That is not a valid rotation - it is not the identity either, which is
    // (0, 0, 0, 1) - and any consumer that normalises it produces NaN and drops
    // the marker. RViz special-cases the zero quaternion, Foxglove's 3D panel
    // does not, which is one way "the path is not visible in Foxglove" while it
    // looks fine in RViz. Say identity explicitly.
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.r = 1.0 - (i * 0.3);
    marker.color.g = 0.2;
    marker.color.b = 0.2 + (i * 0.3);
    marker.color.a = 1.0;

    for (const auto& pt : path)
    {
      geometry_msgs::Point ros_point;
      ros_point.x = pt.x();
      ros_point.y = pt.y();
      ros_point.z = z;
      marker.points.push_back(ros_point);
    }
    marker_array.markers.push_back(marker);
  }
  a_star_path_pub_.publish(marker_array);
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
void EgoPlanner2DNode::generateStraightPath(const PathPoint& start, const PathPoint& goal,
                                            std::vector<PathPoint>& path)
{
  path.clear();

  double dx = goal.x - start.x;
  double dy = goal.y - start.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  int num_points = std::max(2, static_cast<int>(dist / 0.1));

  for (int i = 0; i < num_points; ++i)
  {
    double ratio = static_cast<double>(i) / (num_points - 1);
    PathPoint p;
    p.x = start.x + ratio * dx;
    p.y = start.y + ratio * dy;
    p.z = 0.0;
    p.v = 0.0;
    path.push_back(p);
  }
}

void EgoPlanner2DNode::discretizeTrajectory(const std::vector<PathPoint>& original,
                                            std::vector<PathPoint>& discrete, double interval)
{
  discrete.clear();
  if (original.size() < 2)
    return;

  discrete.push_back(original.front());

  for (size_t i = 0; i < original.size() - 1; ++i)
  {
    const PathPoint& start = original[i];
    const PathPoint& end = original[i + 1];
    double seg_length = distance(start, end);
    if (seg_length < 1e-6)
      continue;

    const int num_points = std::max(
        1, static_cast<int>(std::ceil(seg_length / std::max(interval, 1e-3))));
    for (int j = 1; j <= num_points; ++j)
    {
      const double ratio = static_cast<double>(j) / num_points;
      PathPoint p;
      p.x = start.x + ratio * (end.x - start.x);
      p.y = start.y + ratio * (end.y - start.y);
      p.z = 0.0;
      p.v = 0.0;
      discrete.push_back(p);
    }
  }
}

double EgoPlanner2DNode::distance(const PathPoint& p1, const PathPoint& p2)
{
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  return std::sqrt(dx * dx + dy * dy);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ego_planner_2d");
  EgoPlanner2DNode node;
  ros::spin();
  return 0;
}
