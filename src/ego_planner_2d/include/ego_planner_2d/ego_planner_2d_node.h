/*
 * @Function: 2D Ego-Planner local planner node for ROS1 (melodic / noetic)
 * @Ported from: JackJu-HIT/Ego-Planner-2D-ROS2 (MIT License, develop branch)
 * @Original algorithm: ZJU-FAST-Lab ego-planner
 */
#ifndef EGO_PLANNER_2D_NODE_H_
#define EGO_PLANNER_2D_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <vector>
#include <memory>
#include <mutex>
#include <limits>
#include <string>
#include <unordered_set>

#include <plan_manage/planner_interface.h>

using namespace ego_planner;

class EgoPlanner2DNode
{
public:
  EgoPlanner2DNode();
  ~EgoPlanner2DNode() = default;

private:
  // ---------- callbacks ----------
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  // pose_source == "tf": take the pose from TF (global_frame -> base_frame),
  // e.g. for SLAM backends (cartographer) that only publish TF
  void updatePoseFromTf();
  void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void obstaclesCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void triggerCallback(const std_msgs::Bool::ConstPtr& msg);
  void clearPlan();

  // ---------- planning ----------
  void planningLoop(const ros::TimerEvent& event);
  void replan(const PathPoint& local_pose, const std::vector<PathPoint>& global_path,
              const std::vector<ObstacleInfo>& local_obstacles);
  bool collisionDetection(const std::vector<PathPoint>& traj);

  // ---------- publish ----------
  void publishAll();
  void publishGlobalPath();
  void publishLocalTrajectory();
  void publishObstacles();
  void publishLocalObstacles();
  void publishAStarPaths();
  // Planning is switched off (/trigger_plan == false). Stops replanning but
  // deliberately leaves the last plan published, so the plan stays visible.
  void reportPlanningDisabled();

  // ---------- helpers ----------
  void addObstacleAtPosition(double x, double y);
  // filter + dedup + store one obstacle point; caller must hold data_mutex_
  bool addObstaclePointLocked(double x, double y, double z);
  void generateStraightPath(const PathPoint& start, const PathPoint& goal,
                            std::vector<PathPoint>& path);
  void discretizeTrajectory(const std::vector<PathPoint>& original,
                            std::vector<PathPoint>& discrete, double interval);
  static double distance(const PathPoint& p1, const PathPoint& p2);

  // ---------- ROS ----------
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher global_path_pub_;
  ros::Publisher a_star_path_pub_;
  ros::Publisher local_traj_pub_;
  ros::Publisher obs_pub_;
  ros::Publisher obs_local_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber global_path_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber clicked_point_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Subscriber obstacles_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber trigger_sub_;

  // tf for laser scan -> global frame transformation
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Timer timer_;

  // ---------- planner core ----------
  std::shared_ptr<PlannerInterface> ego_planner_;

  // ---------- state (guarded by data_mutex_) ----------
  std::vector<PathPoint> global_plan_traj_;
  std::vector<ObstacleInfo> obstacles_;
  std::unordered_set<long long> obstacle_cells_; // dedup on quantized grid index
  std::vector<PathPoint> planned_traj_;
  std::vector<std::vector<Eigen::Vector2d>> a_star_pathes_;
  PathPoint cur_pose_; // x, y, z stores yaw
  double cur_z_ = 0.0; // flight height, filled into the published trajectory
  bool has_pose_ = false;
  bool has_valid_global_path_ = false;
  bool should_plan_ = true;
  bool needs_replan_ = false;
  // True once the planning-disabled state has been reported, so it does not
  // re-log at 5 Hz. Reset when planning is switched back on.
  bool stop_reported_ = false;
  std::mutex data_mutex_;

  // ---------- parameters ----------
  double max_vel_ = 2.0;
  double max_acc_ = 3.0;
  double max_jerk_ = 4.0;
  double map_resolution_ = 0.1;
  double map_x_size_ = 50.0;
  double map_y_size_ = 50.0;
  double map_inflate_value_ = 0.5;
  double sensing_range_ = 10.0;
  double planning_rate_ = 5.0;
  double path_sample_interval_ = 0.3;
  double obstacle_height_band_ = 1.0;
  int local_window_points_ = 50;
  int min_traj_points_ = 10;
  // Minimum number of reference points passed to the B-spline optimizer.
  // The optimizer needs > 2 * order_ (= 6) control points, and
  // parameterizeToBspline emits K + 2, so K must exceed 4. Anything smaller is
  // re-sampled at a tighter interval rather than sent to the solver.
  int min_ref_points_ = 8;
  bool obstacle_accumulate_ = true;
  std::string global_frame_ = "map";
  bool pose_source_tf_ = false; // true: pose from TF, false: pose from odom topic
  std::string base_frame_ = "base_link";
  // z written into the published trajectory; < 0 means "use current pose z".
  // Needed for 2D-only localization (cartographer TF has z = 0).
  double output_traj_height_ = -1.0;
};

#endif // EGO_PLANNER_2D_NODE_H_
