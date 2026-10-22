#include "mission_vision.h"

#include <algorithm>
#include <cmath>

MissionVision::MissionVision(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : nh_(nh), private_nh_(private_nh)
{
  private_nh_.param<std::string>("vision_detection_topic", vision_detection_topic_,
                                 "/ssd_detection/detections");
  private_nh_.param<float>("ssd_score_threshold", ssd_score_threshold_, 0.5f);
  private_nh_.param<int>("camera_width", camera_width_, 640);
  private_nh_.param<int>("camera_height", camera_height_, 480);
  private_nh_.param<int>("stable_frames_required", stable_frames_required_, 5);
  private_nh_.param<float>("stable_pixel_tolerance", stable_pixel_tolerance_, 12.0f);

  detection_pub_ = private_nh_.advertise<navigation_2d::TargetDetection>(
      "/mission/vision/detection", 1);
  mode_sub_ = private_nh_.subscribe(
      "/mission/vision/mode", 1, &MissionVision::selectModeCallback, this);
  detection_sub_ = private_nh_.subscribe(
      vision_detection_topic_, 1, &MissionVision::detectionCallback, this);

  ROS_INFO("Mission vision: detections from %s (score >= %.2f)",
           vision_detection_topic_.c_str(),
           static_cast<double>(ssd_score_threshold_));
  // 本节点不开相机、不订阅图像、连 /mission/video/compressed 都不 advertise ——
  // 图像链路整条在检测节点里：真机由它用 GStreamer 直采（MJPEG 硬解 + NVMM
  // 零拷贝），仿真由它订阅 Gazebo 的相机话题；回传画面也只有它那一路
  // （ssd_detection 的 image_topic）。一帧 raw 图都不进 ROS，任务链上只有
  // 检测结果和一路压缩画面。
}

void MissionVision::setMode(Mode mode)
{
  // 这个函数**不许拿锁**。它由任务线程调用（rescue_mission::setVisionMode ←
  // 定时器回调），而任务线程一旦卡在这里，AUTO.LAND 就永远发不出去 ——
  // 2026-10-18 那次"abort 了却一直悬停"就是这个形状：检测线程死锁时持着
  // mutex_，abortMission 里的 setVisionMode(IDLE) 再也返回不了，state_ 停在
  // HAZARD_ALIGN，processMission 的 ABORT 分支（真正 set_mode("AUTO.LAND") 的
  // 地方）根本没机会跑。一个 enum 用一次原子交换就够，稳定帧的清零交给检测线
  // 程在 processDetections 里按帧做（见 last_seen_mode_）。
  if (mode_.exchange(mode) == mode)
  {
    return;
  }
  ROS_INFO("Vision mode: %s", modeName(mode).c_str());
}

MissionVision::Mode MissionVision::mode() const
{
  // 同上：任务线程会问"现在是什么模式"，这里也不能阻塞。
  return mode_.load();
}

void MissionVision::selectModeCallback(const std_msgs::String::ConstPtr &msg)
{
  Mode new_mode;
  if (parseMode(msg->data, new_mode))
  {
    setMode(new_mode);
  }
  else
  {
    ROS_WARN("Unknown vision mode: %s", msg->data.c_str());
  }
}

std::string MissionVision::modeName(Mode mode)
{
  if (mode == HAZARD)
  {
    return "hazard";
  }
  if (mode == PERSON)
  {
    return "person";
  }
  return "idle";
}

bool MissionVision::parseMode(const std::string &value, Mode &mode)
{
  if (value == "hazard")
  {
    mode = HAZARD;
    return true;
  }
  if (value == "person")
  {
    mode = PERSON;
    return true;
  }
  if (value == "idle")
  {
    mode = IDLE;
    return true;
  }
  return false;
}

std::vector<MissionVision::Candidate> MissionVision::findDetectionCandidates(
    int width, int height, Mode mode) const
{
  // mode 由 processDetections 传进来，不在这里读 mode_：一帧之内模式被切走的话，
  // 自己读会出现"按 hazard 筛候选、按 person 贴标签"的错配。
  // detection_mutex_ 仍然要拿，因为 latest_detections_ 会被另一个 spinner 线程的
  // detectionCallback 换掉。⚠️ 前置条件：**调用者不得持有 detection_mutex_**，
  // std::mutex 不可重入。2026-10-18 前 detectionCallback 就是持着这把锁进来的，
  // 症状见那边 swap 处的注释；门禁 16 现在按"锁的生命周期"把这条钉住。
  const bool want_person = (mode == PERSON);

  std::vector<Candidate> candidates;
  int clipped = 0;
  int wrong_zone = 0;
  std::lock_guard<std::mutex> lock(detection_mutex_);
  for (const auto &detection : latest_detections_)
  {
    if (detection.is_person != want_person)
    {
      // 数一遍而已：下面只在"本模式一个候选都没有、但另一类报了 N 个"时打一条，
      // 因为那是任务侧唯一会表现为"识别到了却一直不动"的情形 —— 任务在 timeout
      // 之后只会说 "no stable detection"，从日志里看不出检测器其实一直在报另一个
      // 类。两类常驻混在一个画面里（人员区和危险品区只隔 1.3 m），所以这条要
      // 带着数量打，否则一次误报就会把它变成每 2 s 一句的噪声。
      ++wrong_zone;
      continue;
    }
    const int x1 = static_cast<int>(std::lround(detection.xmin * width));
    const int y1 = static_cast<int>(std::lround(detection.ymin * height));
    const int x2 = static_cast<int>(std::lround(detection.xmax * width));
    const int y2 = static_cast<int>(std::lround(detection.ymax * height));
    const cv::Rect box(x1, y1, x2 - x1, y2 - y1);
    if (box.width < 1 || box.height < 1)
    {
      continue;
    }

    // 贴边的框一定要丢掉。框被画面边界裁掉之后，它的"中心"不再是目标的中心，
    // 而是"可见部分的中心"，也就是朝画面中心偏了一截 —— 拿它算 pixel_dx/dy 会
    // 得到一个方向对、幅度偏小的修正，看起来能飞，实际每次都少修一点。
    // 越靠边偏得越多，而越靠边恰恰越需要修。宁可这一帧不算检测（任务侧会走
    // 视觉超时那条保底路径），也不要拿一个被裁过的框去移动飞机。
    if (box.x <= 0 || box.y <= 0 || box.x + box.width >= width ||
        box.y + box.height >= height)
    {
      ++clipped;
      continue;
    }

    Candidate candidate;
    candidate.box = box;
    // score 是**检测器置信度**，不是面积填充率，所以 TargetDetection.confidence
    // 带上来的就是模型分数。
    candidate.score = detection.score;
    candidates.push_back(candidate);
  }

  if (clipped > 0)
  {
    ROS_WARN_THROTTLE(2.0,
                      "Ignoring %d detection(s) clipped by the frame edge "
                      "(%dx%d): a clipped box's centre is biased toward the "
                      "image centre, so the correction it would produce is "
                      "always too small", clipped, width, height);
  }

  // 一句话把"检测器明明在工作、任务却等不到目标"这条歧义路径钉死。
  // 触发条件是**本模式一个候选都没有**，恰好就是任务侧即将走向 timeout 的那一刻。
  if (candidates.empty() && wrong_zone > 0)
  {
    const char *wanted = want_person ? "person" : "hazard";
    const char *other = want_person ? "hazard" : "person";
    ROS_WARN_THROTTLE(2.0,
                      "The detector reported %d '%s'-zone box(es) and nothing in the "
                      "'%s' zone this mission is waiting for, so found=false. If no "
                      "'%s' box ever arrives the task will end with 'no stable "
                      "detection' after task_timeout. That is a classifier problem "
                      "(class->zone comes from the class name, see labels.txt), not a "
                      "broken image pipeline.", wrong_zone, other, wanted, wanted);
  }
  return candidates;
}

void MissionVision::detectionCallback(
    const ssd_detection::VisionDetectionArray::ConstPtr &message)
{
  std::vector<RawDetection> detections;
  detections.reserve(message->detections.size());
  for (const auto &item : message->detections)
  {
    if (item.confidence < ssd_score_threshold_)
    {
      continue;
    }
    RawDetection detection;
    // 夹到 [0,1]。归一化坐标理论上就在这个范围内，但一个越界的框会让下面
    // 换算出的 cv::Rect 拿到负宽高 —— 而负宽的矩形在框选和
    // cv::norm(center - last_center_) 里都是"看着没事、其实处处不对"的那种。
    detection.xmin = std::min(1.0f, std::max(0.0f, item.xmin));
    detection.ymin = std::min(1.0f, std::max(0.0f, item.ymin));
    detection.xmax = std::min(1.0f, std::max(0.0f, item.xmax));
    detection.ymax = std::min(1.0f, std::max(0.0f, item.ymax));
    detection.score = item.confidence;
    // 按 zone 分家，而不是按类别名。名字的唯一真相源在 labels.py。
    detection.is_person = (item.zone == "person");
    detections.push_back(detection);
  }

  // 锁只包住 swap，**绝不能**跨过下面那次 processDetections()。
  //
  // 2026-10-18 之前它是一个函数级 lock_guard，活到函数末尾，于是
  // detectionCallback → processDetections → findDetectionCandidates 这条链上
  // 同一个线程对 detection_mutex_ 锁了两次。std::mutex 不可重入（UBSan 报
  // recursive lock，glibc 上就是死锁），所以：
  //   * IDLE 分支在自锁之前 return ⇒ 起飞前的心跳全部正常，
  //     require_vision_ready 照样通过、照样解锁起飞；
  //   * 一进 HAZARD/PERSON 模式，第一帧就走 findDetectionCandidates ⇒ 节点
  //     永久卡死，/mission/vision/detection 从此一条不发；
  //   * 任务侧看到的是 last_detection_ 停在 beginTask 里重置的默认值
  //     ⇒ "no stable detection (last: found=0 stable=0)"，而且**一行解释
  //     都没有**：连"丢弃贴边框"的 WARN 都打不出来，因为代码根本走不到。
  //   * Foxglove 里框还在，因为那一路(/ssd_detection/image/compressed)是检测
  //     节点自己画的，跟这里没有任何关系。
  // 用一对花括号把锁的寿命写死，比"记得别锁太久"可靠。
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    latest_detections_.swap(detections);
  }

  // 这条消息就是"一帧来了"。检测节点**每帧都发**，没有目标时发空数组 ——
  // 那不是浪费，是心跳：没有它，本节点不会发布 TargetDetection(found=false)，
  // 任务侧就会一直握着上一次的 found=true，而那可能已经是几秒前的画面
  // （见 rescue_mission 的 task_detection_received_）。心跳同时是起飞前
  // require_vision_ready 门的唯一存活证据。
  processDetections(static_cast<int>(message->image_width),
                    static_cast<int>(message->image_height));
}

void MissionVision::processDetections(int image_width, int image_height)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 一帧只取一次模式快照，下面所有分支都用它。这样即便任务线程在同一刻切模式，
  // 这一帧也是自洽的（不会"按 hazard 扫候选、按 person 贴标签"）。
  const Mode current_mode = mode_.load();
  if (current_mode != last_seen_mode_)
  {
    // 等价于旧 setMode() 里那两行清零，只是搬到了检测线程：切模式后的第一帧
    // 必须从零开始数稳定帧，否则上一轮攒到 5 的计数会让新模式的第一个框立刻
    // 变成 stable=true。搬家的原因是 setMode() 由任务线程调用，它不再拿锁。
    last_seen_mode_ = current_mode;
    stable_frames_ = 0;
    has_last_center_ = false;
  }

  // 帧尺寸随检测消息来；发布方没给（0）时退回本节点的相机参数兜底。
  // 放在 IDLE 判断之前，因为 IDLE 也要用它发那条心跳。
  int width = image_width;
  int height = image_height;
  if (width <= 0 || height <= 0)
  {
    width = camera_width_;
    height = camera_height_;
    ROS_WARN_THROTTLE(30.0,
                      "Vision detection message carries no image size; assuming "
                      "%dx%d from camera_width_/camera_height_. Boxes will be "
                      "misplaced if the detector runs at another resolution.",
                      width, height);
  }

  if (current_mode == IDLE)
  {
    // IDLE 不把候选送到任务侧，但**仍然发一条 found=false**。
    // rescue_mission 的 require_vision_ready 门就是在起飞前靠这条消息判断
    // "检测器已经出帧了"——任务在 HAZARD_HOVER 之前一直是 IDLE，不发这条，
    // 那个门会把一架检测器完全正常的飞机扣在地面。
    //
    // 这条消息顺带把 stable_frames_ 清零、has_last_center_ 置 false（见
    // publishDetection 的 else 分支）。两者等价于 setMode() 每次切模式时的
    // 清空，所以这里多清一遍不改变稳定帧判定的结果。
    publishDetection(width, height, current_mode, nullptr, false);
    return;
  }

  const std::vector<Candidate> candidates = findDetectionCandidates(width, height, current_mode);
  Candidate selected;
  const bool found = selectCandidate(candidates, width, height, selected);
  publishDetection(width, height, current_mode, found ? &selected : nullptr, found);
}

bool MissionVision::selectCandidate(const std::vector<Candidate> &candidates,
                                    int width, int height,
                                    Candidate &out) const
{
  if (candidates.empty())
  {
    return false;
  }

  const float center_x = width * 0.5f;
  const float center_y = height * 0.5f;
  float best = -1.0f;
  for (const auto &candidate : candidates)
  {
    const float dx = candidate.box.x + candidate.box.width * 0.5f - center_x;
    const float dy = candidate.box.y + candidate.box.height * 0.5f - center_y;
    const float distance = std::hypot(dx, dy);
    if (best < 0.0f || distance < best)
    {
      best = distance;
      out = candidate;
    }
  }
  return true;
}

bool MissionVision::publishDetection(int width, int height, Mode mode,
                                     const Candidate *candidate, bool found)
{
  navigation_2d::TargetDetection message;
  message.header.stamp = ros::Time::now();
  message.mode = modeName(mode);
  message.found = found;
  message.image_width = width;
  message.image_height = height;

  if (found && candidate != nullptr)
  {
    const cv::Point2f center(
        candidate->box.x + candidate->box.width * 0.5f,
        candidate->box.y + candidate->box.height * 0.5f);
    const cv::Point2f image_center(width * 0.5f, height * 0.5f);

    message.center_x = center.x;
    message.center_y = center.y;
    message.width = candidate->box.width;
    message.height = candidate->box.height;
    message.pixel_dx = center.x - image_center.x;
    message.pixel_dy = center.y - image_center.y;
    message.confidence = candidate->score;

    if (has_last_center_ &&
        cv::norm(center - last_center_) <= stable_pixel_tolerance_)
    {
      ++stable_frames_;
    }
    else
    {
      stable_frames_ = 1;
    }
    last_center_ = center;
    has_last_center_ = true;
    message.stable = stable_frames_ >= stable_frames_required_;
  }
  else
  {
    stable_frames_ = 0;
    has_last_center_ = false;
    message.stable = false;
  }

  detection_pub_.publish(message);
  return found;
}
