// ssd_detection_node —— Jetson / jetson-inference 版检测节点。
//
// 相机帧进，检测框 + **一路回传画面**出，任务逻辑不在这里（那在
// navigation_2d 的 MissionVision）。与 scripts/ssd_detection_node.py 发的是
// **同一个消息**，所以任务侧不需要知道这一帧是谁算出来的：仿真 PC 上跑
// Python + OpenCV DNN（没有 CUDA），Jetson 上跑这个 C++ + detectNet。
//
// 两种取帧方式（input_URI 空不空）：
//   相机模式（真机，默认）  自己用 GStreamer 抓相机（videoSource，MJPEG 硬解 +
//                            NVMM 零拷贝），不订阅也不发布任何 raw 图像话题；
//   订阅模式（台架/回放）   订阅 input_image_topic，行为与 Python 节点一致。
//
// 本文件看不到任何 jetson-inference / CUDA 类型 —— 全部关在 jetson_detector.cpp 里，
// 见那两个文件的注释。

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <ssd_detection/VisionDetection.h>
#include <ssd_detection/VisionDetectionArray.h>
#include <ssd_detection/jetson_detector.h>

namespace
{

// 红=危险品，蓝=人员。与 MissionVision 里的配色一致，看惯了不会认错。
cv::Scalar colorFor(const std::string& zone)
{
  return zone == "person" ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
}

void drawDetections(cv::Mat& image,
                    const std::vector<ssd_detection::JetsonDetection>& detections)
{
  const int width = image.cols;
  const int height = image.rows;
  for (const auto& detection : detections)
  {
    const cv::Rect box(static_cast<int>(detection.xmin * width),
                       static_cast<int>(detection.ymin * height),
                       static_cast<int>((detection.xmax - detection.xmin) * width),
                       static_cast<int>((detection.ymax - detection.ymin) * height));
    if (box.width < 1 || box.height < 1)
    {
      continue;
    }
    const cv::Scalar color = colorFor(detection.zone);
    cv::rectangle(image, box, color, 3);
    char text[160];
    std::snprintf(text, sizeof(text), "%s %.2f", detection.class_name.c_str(),
                  static_cast<double>(detection.confidence));
    cv::putText(image, text, cv::Point(box.x, std::max(18, box.y - 6)),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
  }
}

// ---------------------------------------------------------------------------
// 离线预览：不启动 ROS、不要 master、不要相机，直接对图片/目录跑推理，
// 把带框的结果写成 JPEG。与 Python 版脚本的 --image/--dir/--out 对称。
//
// 为什么必须有这条路径：换模型、换标签、调阈值之后，"检测效果变了没有" 必须
// 能在台架上静止地看。靠飞起来看，你分不清是检测错了还是飞控错了。而且真机上
// 相机由本节点独占（GStreamer 直采），没有任何 raw 图像话题可订阅，
// 所以台架离线预览是唯一不依赖整条 ROS 链路的验证手段。
//
// 它同时是**唯一能确认 engine 真的建出来了**的地方：节点启动时会把 detectNet
// 缓存下来的 engine 路径打出来。
// ---------------------------------------------------------------------------
struct OfflineOptions
{
  std::string image;
  std::string dir;
  std::string out;
  std::string model;
  std::string labels;
  double score_threshold = 0.5;
};

void printOfflineUsage()
{
  std::fprintf(stderr,
               "ssd_detection_node —— 离线预览（不启动 ROS）\n"
               "  --model  <file>            .onnx 模型（必需）\n"
               "  --labels <file>            标签文件，行 0 = BACKGROUND（必需）\n"
               "  --image <file>             单张图片（与 --dir 二选一）\n"
               "  --dir   <dir>              一个目录，递归找图片\n"
               "  --out   <dir>              输出目录，默认 ./ssd_preview\n"
               "  --score-threshold <f>      默认 0.5\n"
               "\n"
               "不带 --image/--dir 时，照常按 ROS 节点启动。\n"
               "第一次跑会把 ONNX 解析并建 TensorRT engine，缓存到模型旁边，\n"
               "慢是正常的（Nano 上几分钟），之后会直接读缓存。\n");
}

bool parseOfflineArgs(int argc, char** argv, OfflineOptions& opt)
{
  for (int i = 1; i < argc; ++i)
  {
    const std::string key = argv[i];
    const auto take = [&](std::string& dst) {
      if (i + 1 < argc)
      {
        dst = argv[++i];
      }
    };
    if (key == "--image")
    {
      take(opt.image);
    }
    else if (key == "--dir")
    {
      take(opt.dir);
    }
    else if (key == "--out")
    {
      take(opt.out);
    }
    else if (key == "--model")
    {
      take(opt.model);
    }
    else if (key == "--labels")
    {
      take(opt.labels);
    }
    else if (key == "--score-threshold")
    {
      std::string value;
      take(value);
      if (!value.empty())
      {
        opt.score_threshold = std::atof(value.c_str());
      }
    }
    else if (key == "--help" || key == "-h")
    {
      printOfflineUsage();
      std::exit(0);
    }
  }
  return !opt.image.empty() || !opt.dir.empty();
}

std::vector<std::string> collectImages(const OfflineOptions& opt)
{
  if (!opt.image.empty())
  {
    return std::vector<std::string>(1, opt.image);
  }
  std::vector<std::string> found;
  cv::glob(opt.dir + "/*", found, true);

  static const char* kExtensions[] = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"};
  std::vector<std::string> images;
  for (const std::string& file : found)
  {
    std::string lower(file);
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    for (const char* extension : kExtensions)
    {
      const std::size_t length = std::strlen(extension);
      if (lower.size() > length &&
          lower.compare(lower.size() - length, length, extension) == 0)
      {
        images.push_back(file);
        break;
      }
    }
  }
  std::sort(images.begin(), images.end());
  return images;
}

std::string baseName(const std::string& path)
{
  const std::size_t slash = path.find_last_of("/\\");
  const std::string name = (slash == std::string::npos) ? path : path.substr(slash + 1);
  const std::size_t dot = name.find_last_of('.');
  return (dot == std::string::npos) ? name : name.substr(0, dot);
}

int runOffline(const OfflineOptions& opt)
{
  if (opt.model.empty() || opt.labels.empty())
  {
    std::fprintf(stderr,
                 "ssd_detection: --model and --labels are both required in "
                 "offline mode\n");
    printOfflineUsage();
    return 2;
  }

  // 干净的 argv：离线模式下不要把 --image 之类交给 ROS 解析（会被当成 remap 报警）。
  // 这一步只为让 ROS_INFO / ROS_ERROR 有输出，不需要 master（ros::init 本身不连 master）。
  int fake_argc = 1;
  char program_name[] = "ssd_detection";
  char* fake_argv[] = {program_name, nullptr};
  ros::init(fake_argc, fake_argv, "ssd_detection", ros::init_options::NoRosout);

  if (!ssd_detection::jetsonSetup(opt.model, opt.labels,
                                  static_cast<float>(opt.score_threshold)))
  {
    return 3;
  }
  std::printf("engine cache : %s\n",
              ssd_detection::jetsonEnginePath().empty()
                  ? "(none yet)"
                  : ssd_detection::jetsonEnginePath().c_str());

  const std::vector<std::string> files = collectImages(opt);
  if (files.empty())
  {
    ROS_ERROR("ssd_detection: no image found under %s", opt.dir.c_str());
    ssd_detection::jetsonShutdown();
    return 4;
  }

  const std::string out_dir = opt.out.empty() ? std::string("ssd_preview") : opt.out;
  ::mkdir(out_dir.c_str(), 0755);

  std::size_t written = 0;
  for (const std::string& file : files)
  {
    // 不加 const：这份 BGR 就是要被画框、然后 imwrite 出去的那张预览图；
    // 喂模型的是下面 cvtColor 出来的 rgb，两者不共享像素，画框不会污染输入。
    cv::Mat bgr = cv::imread(file, cv::IMREAD_COLOR);
    if (bgr.empty())
    {
      ROS_WARN("ssd_detection: cannot read %s", file.c_str());
      continue;
    }
    // 模型要 RGB，cv::imread 给的是 BGR。这里转一次，与 ROS 路径里 cv_bridge 的
    // "rgb8" 对齐 —— 两条路喂给模型的必须是同一种像素顺序，否则离线预览会与
    // 飞行时的结果不一致，而那种不一致最难发现。
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

    // 包成 ROS 消息复用 jetsonInfer —— 于是实现里不需要为离线模式开第二个入口。
    const sensor_msgs::ImageConstPtr message =
        cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb).toImageMsg();

    const std::vector<ssd_detection::JetsonDetection> detections =
        ssd_detection::jetsonInfer(message);

    std::printf("%s: %zu detection(s)\n", file.c_str(), detections.size());
    for (const auto& detection : detections)
    {
      std::printf("    %-24s %-6s %.3f  [%.3f %.3f %.3f %.3f]\n",
                  detection.class_name.c_str(), detection.zone.c_str(),
                  static_cast<double>(detection.confidence),
                  static_cast<double>(detection.xmin),
                  static_cast<double>(detection.ymin),
                  static_cast<double>(detection.xmax),
                  static_cast<double>(detection.ymax));
    }

    drawDetections(bgr, detections);
    std::ostringstream name;
    name << out_dir << "/" << baseName(file) << "_det.jpg";
    if (cv::imwrite(name.str(), bgr))
    {
      ++written;
    }
    else
    {
      ROS_WARN("ssd_detection: cannot write %s", name.str().c_str());
    }
  }

  std::printf("ssd_detection: %zu image(s) processed, %zu written to %s\n",
              files.size(), written, out_dir.c_str());
  ssd_detection::jetsonShutdown();
  return 0;
}

}  // namespace

class SsdDetectionNode
{
public:
  SsdDetectionNode()
    : private_nh_("~")
  {
    // ---- 相机（GStreamer 直采）------------------------------------------
    // 非空 ⇒ 相机模式：本节点自己抓帧，**不订阅任何图像话题**。真机走这条：
    // videoSource 在 Jetson 上是 v4l2src ! nvv4l2decoder(mjpeg) ! nvvidconv !
    // NVMM ! appsink，MJPEG 硬件解码、终点是映射到 CPU 的零拷贝显存，一帧图
    // 从头到尾不进 ROS、不经过 usb_cam、不被第二个订阅者再拷一次。
    // 空 ⇒ 订阅模式（input_image_topic），留给台架回放 / rosbag / 别的发布者。
    private_nh_.param<std::string>("input_URI", input_uri_, "v4l2:///dev/video0");
    private_nh_.param<int>("input_width", input_width_, 640);
    private_nh_.param<int>("input_height", input_height_, 480);
    private_nh_.param<int>("input_rate", input_rate_, 15);
    // mjpeg = 相机的 MJPEG 流（USB2 带宽下 640x480@15 基本只有它稳）。
    // 换成 yuyv/raw 那种未压缩流要把宽度帧率往下让 —— USB2 带不动。
    private_nh_.param<std::string>("input_codec", input_codec_, "mjpeg");
    private_nh_.param<std::string>("input_image_topic", input_topic_, "/camera/image_raw");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/ssd_detection/detections");
    private_nh_.param<std::string>("model_path", model_path_, "");
    private_nh_.param<std::string>("label_file", label_file_, "");
    // 唯一的回传画面：画了框的检测结果（红=危险品，蓝=人员）。ssd 后端下
    // 整条任务链里**只有这一路图像话题** —— 没有 raw 话题，也没有第二路
    // 调试图。关掉它（false）只在纯台架、没人看画面时才有意义。
    private_nh_.param<std::string>("image_topic", image_topic_,
                                   "/ssd_detection/image/compressed");
    private_nh_.param<bool>("publish_image", publish_image_, true);
    // 回传前先把长边压到这个宽度（0 = 原尺寸）。相机 1280x720 而出图 640
    // 是弱链路下的常规操作；检测始终在原图上跑，不受这个值影响。
    private_nh_.param<int>("video_width", video_width_, 640);
    // 阈值交给 detectNet 的 mConfidenceThreshold，飞行中改它要重启节点。
    private_nh_.param<double>("score_threshold", score_threshold_, 0.5);
    private_nh_.param<int>("jpeg_quality", jpeg_quality_, 70);
    // 推理限速（Hz）。0 = 不限制，来多少帧算多少帧。
    // 相机端本来就有节流（GStreamer 的 framerate），这里是第二道：万一图像源
    // 更快，Jetson Nano 会在推理上把 CPU 吃满，而丢帧这件事必须由我们说了
    // 算、不是由积压说了算。
    private_nh_.param<double>("max_rate", max_rate_, 0.0);
    // 检测消息/回传画面的 frame_id。真机上没有相机的 TF，这个值纯属日志与
    // Foxglove 面板的观感（与当年 usb_cam 的 camera_link 一致）。
    private_nh_.param<std::string>("camera_frame_id", camera_frame_id_, "camera_link");
  }

  bool cameraMode() const { return camera_mode_; }

  // 公开：相机模式由 main() 直接驱动；订阅模式走 ros::spin，不用它。
  // 相机模式主循环：来一帧算一帧。
  //
  //   Capture() 阻塞到有新帧（或 1000 ms 超时）⇒ 循环天然以相机帧率运行；
  //   appsink 只留最新一帧 ⇒ 推理比相机慢时我们处理的始终是最新画面，
  //   不会出现"框越来越旧"的积压。
  //   max_rate>0 时再压一道下限：主动丢帧，而不是让 Nano 在推理上吃满 CPU
  //   （cartographer 还在同一个 Nano 上跑）。
  void runCameraLoop()
  {
    cv::Mat rgb;  // 相机缓冲的浅视图：每帧复用，不分配
    cv::Mat bgr;  // 画框用（RGB->BGR 的目标）
    ros::Time cycle_start;

    while (ros::ok())
    {
      if (max_rate_ > 0.0)
      {
        cycle_start = ros::Time::now();
      }

      if (!ssd_detection::jetsonCameraCapture(rgb, 1000))
      {
        if (!ssd_detection::jetsonCameraIsStreaming())
        {
          // 设备被拔/驱动复位：GStreamer 已经关流。继续转下去只会每秒钟
          // 超时一次，而任务侧握着过期目标比没有目标更危险 —— 退出，让
          // mission 按 vision_with_fallback 退回写死航点，日志里也看得见。
          ROS_FATAL("ssd_detection: camera stream ended; exiting so the mission "
                    "falls back to waypoint drops instead of trusting a dead "
                    "camera");
          return;
        }
        ROS_WARN_THROTTLE(5.0, "ssd_detection: no camera frame within 1000 ms");
        continue;
      }

      const std::vector<ssd_detection::JetsonDetection> detections =
          ssd_detection::jetsonInferRgb8(
              reinterpret_cast<const uint8_t*>(rgb.data), rgb.cols, rgb.rows);

      // 检测数组：任务侧（MissionVision）的唯一输入。**空数组也要发** ——
      // 它是"这一帧没有目标"的心跳：没有它，MissionVision 不会发布
      // TargetDetection(found=false)，任务侧就会一直握着上一次的
      // found=true，而那可能已经是几秒前的画面了。
      ssd_detection::VisionDetectionArray array;
      array.header.stamp = ros::Time::now();
      array.header.frame_id = camera_frame_id_;
      array.image_width = static_cast<uint32_t>(rgb.cols);
      array.image_height = static_cast<uint32_t>(rgb.rows);
      array.detections.reserve(detections.size());
      for (const auto& detection : detections)
      {
        ssd_detection::VisionDetection item;
        item.class_name = detection.class_name;
        item.zone = detection.zone;
        item.confidence = detection.confidence;
        item.xmin = detection.xmin;
        item.ymin = detection.ymin;
        item.xmax = detection.xmax;
        item.ymax = detection.ymax;
        array.detections.push_back(item);
      }
      detection_pub_.publish(array);

      if (publish_image_)
      {
        // 相机缓冲是 RGB（videoSource 的 IMAGE_RGB8）；画框与 JPEG 要 BGR。
        // 这一次转换是每帧唯一的一次全图拷贝，之后画框/缩放都在 bgr 上做。
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        drawDetections(bgr, detections);
        encodeAndPublish(bgr, array.header);
      }

      if (max_rate_ > 0.0)
      {
        const ros::Duration period(1.0 / max_rate_);
        const ros::Duration elapsed = ros::Time::now() - cycle_start;
        if (elapsed < period)
        {
          (period - elapsed).sleep();
        }
      }
    }
  }

  bool start()
  {
    if (!ssd_detection::jetsonSetup(model_path_, label_file_,
                                    static_cast<float>(score_threshold_)))
    {
      // 明确的失败：与其带着一个永远不出发的检测节点起飞，不如现在退出。
      ROS_FATAL("ssd_detection: detectNet setup failed; see the errors above");
      return false;
    }
    ROS_INFO("ssd_detection: label file has %d line(s) (line 0 must be BACKGROUND). "
             "detectNet reads the class count from the model, not from this file, "
             "so run tools/ssd_training/check_onnx_contract.py before trusting them.",
             ssd_detection::jetsonLabelCount());

    detection_pub_ =
        private_nh_.advertise<ssd_detection::VisionDetectionArray>(output_topic_, 1);
    if (publish_image_)
    {
      image_pub_ =
          private_nh_.advertise<sensor_msgs::CompressedImage>(image_topic_, 1);
    }

    if (!input_uri_.empty())
    {
      // 相机模式。**模型就绪之后才开相机**：Nano 上首次建 TensorRT engine 要
      // 几十秒（2026-10-07 实机记录：26.5 s），那期间让 GStreamer 占着设备
      // 空转毫无意义，更糟的是会让任务侧以为视觉已经好了。
      if (!ssd_detection::jetsonCameraOpen(input_uri_, input_width_, input_height_,
                                           input_rate_, input_codec_))
      {
        // 明确的失败：与其带着一个永远不出发的检测节点起飞，不如现在退出。
        // 任务侧拿不到任何检测，会按 vision_with_fallback 退回写死航点投放 ——
        // 那正是相机打不开时应该发生的事。
        ROS_FATAL("ssd_detection: camera '%s' failed to open; see the errors above",
                  input_uri_.c_str());
        return false;
      }
      camera_mode_ = true;
    }
    else
    {
      // queue_size=1：检测慢于相机帧率时，宁可丢掉旧帧也不要排队。
      // 排队的后果是"框所在的时刻"越来越落后于"图上的时刻"，
      // 而 MissionVision 正是拿框的中心当作当下目标的位置。
      image_sub_ = private_nh_.subscribe(input_topic_, 1, &SsdDetectionNode::onImage,
                                         this);
    }

    ROS_INFO("ssd_detection (detectNet): %s -> %s", input_topic_.c_str(),
             output_topic_.c_str());
    if (camera_mode_)
    {
      ROS_INFO("ssd_detection: camera mode, capturing %s directly via GStreamer "
               "(no image topic is subscribed or published raw)",
               input_uri_.c_str());
    }
    if (max_rate_ > 0.0)
    {
      ROS_INFO("ssd_detection: inference capped at %.1f Hz", max_rate_);
    }
    if (publish_image_)
    {
      ROS_INFO("ssd_detection: detection feed on %s", image_topic_.c_str());
    }
    return true;
  }

private:
  void onImage(const sensor_msgs::ImageConstPtr& message)
  {
    // 限速：丢掉超额帧。queue_size=1 只是不积压，这一层是"算得比看得快"时
    // 主动降频，让 CPU/GPU 有余量给飞控那条链路。
    if (max_rate_ > 0.0)
    {
      const ros::Time now = ros::Time::now();
      if (!last_infer_.isZero() && (now - last_infer_).toSec() < 1.0 / max_rate_)
      {
        return;
      }
      last_infer_ = now;
    }

    // 内部已做 cv_bridge 异常处理与通道序（rgb8），这里不再重复包 try。
    const std::vector<ssd_detection::JetsonDetection> detections =
        ssd_detection::jetsonInfer(message);

    ssd_detection::VisionDetectionArray array;
    array.header = message->header;  // 用原来那一帧的时间戳，不是 now()
    // 帧尺寸随消息走：任务侧（MissionVision）靠它把归一化框换算成像素，
    // 不能假设它相机参数里的 640x480 就是这一帧的尺寸。
    array.image_width = static_cast<uint32_t>(message->width);
    array.image_height = static_cast<uint32_t>(message->height);
    array.detections.reserve(detections.size());
    for (const auto& detection : detections)
    {
      ssd_detection::VisionDetection item;
      item.class_name = detection.class_name;
      item.zone = detection.zone;
      item.confidence = detection.confidence;
      item.xmin = detection.xmin;
      item.ymin = detection.ymin;
      item.xmax = detection.xmax;
      item.ymax = detection.ymax;
      array.detections.push_back(item);
    }
    detection_pub_.publish(array);

    if (publish_image_)
    {
      publishImage(message, detections);
    }
  }

  // 订阅模式的回传画面：从 ROS 消息取帧画框。相机模式不走这里（见
  // runCameraLoop）—— 那边帧在相机缓冲里，包成消息再解一次纯属浪费。
  void publishImage(const sensor_msgs::ImageConstPtr& message,
                    const std::vector<ssd_detection::JetsonDetection>& detections)
  {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvCopy(message, "bgr8")->image;
    }
    catch (const cv_bridge::Exception& error)
    {
      ROS_WARN_THROTTLE(5.0, "ssd_detection: cv_bridge: %s", error.what());
      return;
    }
    drawDetections(image, detections);
    encodeAndPublish(image, message->header);
  }

  void encodeAndPublish(const cv::Mat& image, const std_msgs::Header& header)
  {
    if (image.empty())
    {
      return;
    }

    cv::Mat output = image;
    if (video_width_ > 0 && image.cols > video_width_)
    {
      const int height = static_cast<int>(
          static_cast<double>(image.rows) * video_width_ / image.cols);
      cv::resize(image, output, cv::Size(video_width_, height));
    }

    std::vector<int> encode_params;
    encode_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    encode_params.push_back(std::max(20, std::min(95, jpeg_quality_)));

    std::vector<uchar> encoded;
    if (!cv::imencode(".jpg", output, encoded, encode_params))
    {
      ROS_WARN_THROTTLE(5.0, "ssd_detection: JPEG encode failed");
      return;
    }

    sensor_msgs::CompressedImage picture;
    picture.header = header;
    picture.format = "jpeg";
    picture.data.assign(encoded.begin(), encoded.end());
    image_pub_.publish(picture);
  }

  ros::NodeHandle private_nh_;
  ros::Publisher detection_pub_;
  ros::Publisher image_pub_;
  ros::Subscriber image_sub_;

  bool camera_mode_ = false;

  std::string input_uri_;
  int input_width_ = 640;
  int input_height_ = 480;
  int input_rate_ = 15;
  std::string input_codec_;
  std::string input_topic_;
  std::string output_topic_;
  std::string image_topic_;
  std::string model_path_;
  std::string label_file_;
  double score_threshold_ = 0.5;
  bool publish_image_ = true;
  int video_width_ = 640;
  int jpeg_quality_ = 70;
  double max_rate_ = 0.0;
  std::string camera_frame_id_;
  // 订阅模式的限速时刻（相机模式用循环周期，见 runCameraLoop）。
  ros::Time last_infer_;
};

int main(int argc, char** argv)
{
  // 先看是不是离线预览（--image / --dir）。是的话完全不碰 ROS 的节点部分：
  // 不要 master、不要相机，跑完就退出。这不是"顺带支持" —— 台架验证必须能
  // 在整条链路之外单独跑，否则检测器坏了和链路坏了分不开。
  OfflineOptions offline;
  if (parseOfflineArgs(argc, argv, offline))
  {
    return runOffline(offline);
  }

  ros::init(argc, argv, "ssd_detection");

  SsdDetectionNode node;
  if (!node.start())
  {
    return 1;
  }

  if (node.cameraMode())
  {
    // 相机模式：主循环在本线程里跑（ Capture 阻塞即节流），没有任何订阅
    // 需要 spin。
    node.runCameraLoop();
  }
  else
  {
    ros::spin();
  }
  ssd_detection::jetsonShutdown();
  return 0;
}
