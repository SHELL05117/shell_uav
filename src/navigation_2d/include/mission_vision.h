#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>

#include <navigation_2d/TargetDetection.h>
// 检测结果从独立的 ssd_detection 包来。这里只依赖它的消息，不依赖它的节点：
// 仿真里跑 ssd_detection_node.py（OpenCV DNN，订阅 Gazebo 相机话题），
// 实机上跑同一个包的 C++ detectNet 节点（jetson-inference，GStreamer 直采
// 相机），两边都发这个类型，所以下面的逻辑两边一模一样。
//
// 2026-10-17：HSV 色窗后端已删除。这个节点不再开相机、不订阅图像、不发
// /mission/video/compressed —— 图像链路整条归检测节点，本节点只消费检测结果。
// 于是这里不再有 cv_bridge，也不再需要 <sensor_msgs/Image.h>。
#include <ssd_detection/VisionDetectionArray.h>

class MissionVision
{
public:
  enum Mode
  {
    IDLE,
    HAZARD,
    PERSON
  };

  MissionVision(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

  void setMode(Mode mode);
  Mode mode() const;

private:
  struct Candidate
  {
    cv::Rect box;
    float score = 0.0f;
  };

  // 缓存的原始检测框。存**归一化**坐标而不是像素：回调拿到的只是检测消息，
  // 没有那一帧图像，尺寸随消息的 image_width/image_height 一起来
  // （0 = 发布方没给，退回 camera_width_/height_ 参数）。
  // 也存 zone 而不是类别名 —— 类别名的唯一真相源在 labels.py，抄进 C++ 就会
  // 在改名字的时候漏一处。
  struct RawDetection
  {
    float xmin = 0.0f;
    float ymin = 0.0f;
    float xmax = 0.0f;
    float ymax = 0.0f;
    float score = 0.0f;
    bool is_person = false;
  };

  void selectModeCallback(const std_msgs::String::ConstPtr &msg);
  // 检测节点每帧发一条（空数组 = 这一帧没有目标的心跳），所以处理节奏与相机
  // 帧率一致。image_width/height 是那条消息带来的帧尺寸，0 表示发布方没给。
  void detectionCallback(const ssd_detection::VisionDetectionArray::ConstPtr &message);
  void processDetections(int image_width, int image_height);
  // `mode` 是调用方在一帧开头读到的快照，不是现读 mode_：否则一帧里模式被切走
  // 会出现"按 hazard 扫候选、按 person 贴标签"的错配消息。
  std::vector<Candidate> findDetectionCandidates(int width, int height,
                                                 Mode mode) const;
  // 取**离画面中心最近**的那个候选（2026-10-17 起）。
  //
  // 以前是按 hazard_target_index / person_target_index 取第几个，那两个常量
  // 需要有人手工记住"检测器从上到下排出来第几个才是脚下那张卡"，而错了没有
  // 任何报错 —— 只会投到隔壁卡片上。相机朝下、飞机悬在目标上方，所以"离画面
  // 中心最近"就是"离飞机最近"，这是一个几何事实而不是一个约定。
  //
  // 返回 false 表示没有可用候选（out 不动）。
  bool selectCandidate(const std::vector<Candidate> &candidates, int width,
                       int height, Candidate &out) const;
  // 稳定帧判定 + TargetDetection。`mode` 同样是快照，直接写进消息的 mode 字段。
  bool publishDetection(int width, int height, Mode mode,
                        const Candidate *candidate, bool found);
  static std::string modeName(Mode mode);
  static bool parseMode(const std::string &value, Mode &mode);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher detection_pub_;
  ros::Subscriber mode_sub_;
  ros::Subscriber detection_sub_;

  std::string vision_detection_topic_ = "/ssd_detection/detections";
  float ssd_score_threshold_ = 0.5f;
  // 只在检测消息没带帧尺寸（image_width/height 为 0）时兜底。
  int camera_width_ = 640;
  int camera_height_ = 480;
  int stable_frames_required_ = 5;
  float stable_pixel_tolerance_ = 12.0f;

  // 模式是**原子**的，不是 mutex_ 保护的，这条是安全相关的：
  // setMode() 由任务线程调用（rescue_mission 的定时器回调 → setVisionMode），
  // 而任务线程一旦阻塞在这里，AUTO.LAND 就永远发不出去 —— 2026-10-18 那次
  // "abort 打印了却一直悬停"正是这个形状：检测线程死锁时持着 mutex_，abort
  // 路径里的 setVisionMode(IDLE) 就再也返回不了，state_ 停在 HAZARD_ALIGN，
  // processMission 的 ABORT 分支根本没机会跑。
  // 任务线程调用本层的任何函数都**不许拿锁**。稳定帧状态（stable_frames_ /
  // has_last_center_ / last_center_ / last_seen_mode_）只由检测线程按帧访问，
  // 由 mutex_ 串行化；切模式时的清零搬到 processDetections 里做。
  std::atomic<Mode> mode_{IDLE};
  int stable_frames_ = 0;
  bool has_last_center_ = false;
  cv::Point2f last_center_;
  // 检测线程上一帧看到的模式。与 mode_ 不同 ⇒ 清零稳定帧计数，等价于旧
  // setMode() 里那两行（否则切模式后会继承上一轮攒的 stable_frames_）。
  Mode last_seen_mode_ = IDLE;
  // 只保护上面那四个字段，已经没有第二个线程会来拿它了。
  mutable std::mutex mutex_;

  // 检测缓存有**自己**的锁，不跟 mutex_ 共用。节点用的是 AsyncSpinner(3)，
  // 检测回调与模式回调是真并发。
  //
  // 刻意用 std::mutex 而不是 recursive_mutex：这把锁曾经在同一个线程里被锁
  // 两次（detectionCallback 持锁调 processDetections，后者进
  // findDetectionCandidates 又锁一次），节点在切到 HAZARD/PERSON 模式的第一帧
  // 就静默死锁。recursive_mutex 能让那个形状"跑起来"，但读写竞态仍然存在，只
  // 是把死锁换成了更难查的东西。锁的粒度见 mission_vision.cpp 里 swap 处的花
  // 括号；门禁 16 会检查调用点不在任何内层作用域里。
  std::vector<RawDetection> latest_detections_;
  mutable std::mutex detection_mutex_;
};
