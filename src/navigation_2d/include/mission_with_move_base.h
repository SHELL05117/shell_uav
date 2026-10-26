/***************************************************************************************************************************
 *
 * Author: expload
 * Time: 2026.10
 * Description: 2026 消防与救援任务的机载任务状态机。
 *
 *   起飞 -> 危险品任务区（两张卡逐个识别，正确目标红框标注，投消防物资）
 *        -> 人员救助任务区（三张卡逐个识别，伤员蓝框标注，决赛投救助物资）
 *        -> 返航 -> 摄像头回正 -> 自动降落
 *
 * 航点全部以“场地系”给出（原点=起飞点 H，x 向图纸右，y 向图纸上，米），
 * 启动时按 field_yaw_deg 旋到 map 系再交给 move_base。机头朝场地系 -y，
 * 实机和仿真都取 field_yaw_deg=-90（仿真里把场地在 world 里转了 90°，所以
 * map = world 的 +x 也是机头方向，见 docs/field_frame.md 第 2 节）。
 *
 * DWA速度按实际机体航向旋入PX4本地系。map与PX4本地系使用同时段机体观测
 * 建立平面变换，不要求两者原点或航向天然重合。默认禁用实际控制输出。
 ***************************************************************************************************************************/
#ifndef NAVIGATION_2D_MISSION_WITH_MOVE_BASE_H
#define NAVIGATION_2D_MISSION_WITH_MOVE_BASE_H

#include <deque>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/State.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "detect.h"
#include "flight_control.h"
#include "dwa_control_math.h"
#include "dwa_mission_policy.h"

class MoveBaseMission
{
public:
  MoveBaseMission(const ros::NodeHandle &nh_private, int argc, char **argv);
  ~MoveBaseMission();

private:
  struct FieldPoint
  {
    double x = 0.0;
    double y = 0.0;
  };

  struct Waypoint
  {
    FieldPoint field;
    double map_x = 0.0;
    double map_y = 0.0;
    std::string name;
    bool detect = false;
  };

  struct Hit
  {
    ros::Time stamp;
    cv::Point2f center;  // 源图像素
    Detect::Detection det;
  };

  enum class Phase
  {
    WAIT_FCU,
    TAKEOFF,
    TRANSIT,
    SETTLE,
    DETECT,
    DESCEND,
    DROP,
    CLIMB,
    LAND,
    DONE
  };

  enum class Task
  {
    HAZARD,
    PERSON,
    RETURN
  };

  enum class ControlMode
  {
    HOLD,
    TRACK,
    IDLE
  };

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  // --- callbacks / loops ---------------------------------------------------
  void StateCallback(const mavros_msgs::State &msg);
  void PoseCallback(const geometry_msgs::PoseStamped &msg);
  void PlannerVelCallback(const geometry_msgs::Twist &msg);
  void AckCallback(const std_msgs::String &msg);
  void TargetCallback(const std_msgs::String &msg);
  void ControlLoop(const ros::TimerEvent &event);
  void TickLoop(const ros::TimerEvent &event);

  // --- setup ---------------------------------------------------------------
  void loadParams(int argc, char **argv);
  void buildRoute(const std::string &prefix, const std::string &count_key,
                  const std::string &detect_key, std::vector<Waypoint> &route);

  // --- state machine -------------------------------------------------------
  void beginTakeoff();
  void startTask(Task task);
  void nextLeg();
  void onArrived();
  void beginSettle(const Waypoint &waypoint);
  void beginDetect();
  void onConfirmed(const Detect::Detection &detection, const cv::Point2f &body_offset);
  void onDetectFailed();
  void beginDrop(const geometry_msgs::Point &target, bool fire);
  void beginClimb();
  void beginLand();
  void finishMission();

  // --- helpers -------------------------------------------------------------
  void sendGoal(const Waypoint &waypoint);
  void holdAt(const geometry_msgs::Point &point);
  void holdCurrent();
  void sendGpio(const std::string &command);
  void setStatus(const std::string &status);
  void requestMode(const std::string &mode);
  bool poseFresh() const;
  bool stateFresh() const;
  bool updateFrameAlignment();
  bool mapPointToLocal(const FieldPoint &map, geometry_msgs::Point &local) const;
  void blockMission(const std::string &reason, const ros::Time &now);
  void abortMission(const std::string &reason);
  bool requiresFlightAuthority() const;
  bool flightAuthorityAllowed(const mavros_msgs::State &state) const;
  bool checkFlightAuthority();
  void resumeMission(const ros::Time &now);
  void publishReadiness(bool model_ok, bool pose_ok, bool state_ok, bool frame_ok);
  bool refreshDetectTarget(Detect::Detection &best, cv::Point2f &body_offset);
  std::string resolveHazardTarget(const std::string &text) const;

  bool atHeight(double z, double tolerance) const;
  double yaw() const;
  geometry_msgs::PoseStamped poseSnapshot() const;
  mavros_msgs::State stateSnapshot() const;
  geometry_msgs::Point bodyToLocal(const cv::Point2f &body) const;
  FieldPoint fieldToMap(const FieldPoint &field) const;
  FieldPoint clampField(const FieldPoint &field) const;
  const std::vector<Waypoint> &currentRoute() const;

  static double yawOf(const geometry_msgs::Quaternion &q);
  static void logRoute(const std::vector<Waypoint> &route, const char *name, double yaw_deg);

  // --- ROS -----------------------------------------------------------------
  ros::NodeHandle nh_private_;
  ros::Subscriber state_sub_, pose_sub_, cmd_vel_sub_, ack_sub_, target_sub_;
  ros::Publisher status_pub_, gpio_cmd_pub_;
  ros::Publisher readiness_pub_;
  ros::Timer control_timer_, tick_timer_;

  MoveBaseClient move_base_;
  bool move_base_ready_ = false;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Detect detect_;
  FlightControl flightControl;

  // --- parameters ----------------------------------------------------------
  double fly_height_ = 1.5;
  double detect_height_ = 1.2;
  double drop_height_ = 0.6;
  double takeoff_height_tolerance_ = 0.12;
  double takeoff_timeout_ = 20.0;
  double settle_time_ = 1.0;
  double goal_timeout_ = 25.0;
  double drop_settle_time_ = 0.8;
  double ack_timeout_ = 3.0;
  double land_settle_time_ = 2.0;
  double max_vel_ = 0.35;
  double xy_gain_p_ = 1.2;
  double z_gain_p_ = 1.0;
  double pub_rate_ = 50.0;

  double field_yaw_deg_ = -90.0;
  bool enforce_safe_field_ = true;
  double field_min_x_ = -0.55, field_max_x_ = 3.25;
  double field_min_y_ = -3.25, field_max_y_ = 0.55;

  bool auto_start_ = false;
  bool enable_actuation_ = false;
  double pose_timeout_ = 0.5;
  double state_timeout_ = 3.0;
  double tf_timeout_ = 0.5;
  double planner_timeout_ = 0.5;
  double alignment_tolerance_ = 0.2;
  std::string map_frame_ = "map";
  std::string body_frame_ = "base_link";
  int stage_ = 1;
  bool rescue_drop_enabled_ = false;
  bool rescue_drop_ = false;

  std::string target_hazard_;
  std::string target_person_;
  std::string target_topic_ = "/mission/target_hazard";
  bool wait_for_target_ = true;
  int detect_times_ = 5;
  double detect_timeout_ = 8.0;
  double detect_confirmation_window_ = 6.0;
  double annotate_hold_ = 4.0;
  double max_center_offset_ = 0.35;

  std::string drop_align_mode_ = "vision_with_fallback";
  double max_align_offset_ = 0.35;
  FieldPoint rescue_drop_field_;
  bool rescue_drop_field_set_ = false;

  int hazard_detect_num_ = 0;
  int person_detect_num_ = 0;

  // --- runtime -------------------------------------------------------------
  Phase phase_ = Phase::WAIT_FCU;
  Task task_ = Task::HAZARD;
  ControlMode control_mode_ = ControlMode::HOLD;
  ros::Time phase_start_;
  ros::Time last_mode_request_;
  ros::Time last_arm_request_;

  std::vector<Waypoint> hazard_route_, person_route_, return_route_;
  size_t leg_ = 0;
  Waypoint current_waypoint_;

  mavros_msgs::State fcu_state_;
  bool fcu_connected_ = false;
  bool has_pose_ = false;
  double hold_yaw_ = 0.0;
  geometry_msgs::PoseStamped current_pose_;
  mutable std::mutex state_mutex_, pose_mutex_, target_mutex_;
  geometry_msgs::Point target_position_;  // local/odom 系，给位置环
  geometry_msgs::Point takeoff_origin_;
  // DWA body-frame command, rotated using current local yaw at publication.
  geometry_msgs::Point planner_velxy_posz_;
  ros::Time planner_command_stamp_;
  mutable std::recursive_mutex runtime_mutex_;
  dwa_control::Pose2 map_body_, local_body_;
  ros::Time alignment_stamp_;
  bool frame_alignment_valid_ = false;
  bool runtime_blocked_ = false;
  bool fault_hold_valid_ = false;
  geometry_msgs::Point fault_hold_position_;
  ros::Time blocked_since_;
  std::string last_status_;
  // An abort is irreversible for this node instance. WAIT_FCU preparation and
  // deliberate LAND mode changes are separate from active flight authority.
  std::string mission_abort_reason_;

  bool goal_active_ = false;
  ros::Time goal_sent_;
  int goal_attempts_ = 0;

  std::deque<Hit> hits_;
  uint64_t last_detection_sequence_ = 0;
  ros::Time last_detect_log_;
  bool drop_is_fire_ = true;
  geometry_msgs::Point drop_target_;
  bool drop_sent_ = false;
  ros::Time drop_cmd_time_;
  bool ack_received_ = false;
  dwa_mission::DropKind drop_authorization_ = dwa_mission::DropKind::None;
  std::string hazard_detection_ = "pending";
  std::string person_detection_ = "pending";
  Task after_drop_task_ = Task::PERSON;

  int land_stage_ = 0;
  ros::Time land_stage_time_;
  bool land_warned_ = false;
};

#endif  // NAVIGATION_2D_MISSION_WITH_MOVE_BASE_H
