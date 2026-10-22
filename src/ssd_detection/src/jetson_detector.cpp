// jetson_detector.cpp —— detectNet 的实现。
//
// 本文件是**唯一**能看到 jetson-inference / CUDA 头文件的地方；对外只给
// ssd_detection/jetson_detector.h 里那组自由函数。见那个文件的顶部注释。
//
// 与被删掉的 ssd_trt_infer.cu 的差别，说白了只有一句：**谁来解析这个模型**。
// 那一版是我们自己 deserializeCudaEngine，所以要自己注册 FlattenConcat_TRT；
// 这一版交给 detectNet，它读 ONNX、自己建 engine、自己缓存，插件的事不存在了。
//
// 三条从 detectNet 源码读来的、必须照做的约定：
//   1. 预处理在 detectNet::preProcess() 里，ONNX 分支走 cudaTensorNormMeanRGB：
//      **RGB** 通道序、(x/255-0.5)/0.5 ∈ [-1,1]。UFF 分支是 BGR —— 数值范围相同、
//      通道序相反，所以本文件喂进去的必须是 RGB，cv_bridge 的 "rgb8" 正好。
//   2. 后处理走 postProcessSSD_ONNX：scores/boxes 两个输出，class 0 是 BACKGROUND
//      被跳过，boxes 已经解码成归一化的角点（左上、右下），**不做 [0,1] 裁剪**。
//   3. 类数 mNumClasses = scores 最后一维，是从**张量**读出来的；labels.txt 只
//      提供名字，两者从不核对。少一行就静默零检出 —— 靠 check_onnx_contract.py
//      在本机上提前查，本文件只能把 label 行数打出来。
//   4. blob 名是命令行参数、不是图属性：detectNet 按 --input-blob（这个版本
//      默认 'data'）/ --output-cvg / --output-bbox 的名字在**建好的 engine** 里
//      查 binding，查不到输入就报 "failed to find requested input layer data
//      in network" 并放弃。所以人造 argv 必须显式带上这三个名字。
//   5. 相机走 videoSource（GStreamer）：v4l2:// 在 Jetson 上是
//      v4l2src ! jpegparse ! nvv4l2decoder mjpeg=1 ! nvvidconv ! NVMM ! appsink，
//      MJPEG 硬件解码、终点 NVMM，Capture() 交回 CPU 映射的零拷贝指针。
//      **注意版本差异**：新版 jetson-inference 的 gst_select_decoder() 对 MJPEG
//      强制 CODEC_CPU（jpegdec 软解）；实机 Nano 上装的那版没有这个强制，走
//      nvv4l2decoder 硬解。两种都能跑，只是 CPU 占用不同 —— 启动时 gstCamera
//      会把 pipeline string 打进日志，现场核对那一行就知道走了哪条。

#include "ssd_detection/jetson_detector.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/stat.h>
#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// jetson-inference 的 C++ 类都在**全局命名空间**（不包在 jetson-inference 里），
// 所以这里不带命名空间限定 —— 与旧工程 detect.cpp 的写法一致，那份在实机上编过。
#include "jetson-inference/detectNet.h"

// videoSource.h 的安装目录**随版本变过一次**，硬编码哪一条都会在另一种机器上
// 编不过（实机第一次编就栽在这一行）：
//   * utils 还内嵌在 jetson-inference 里的老版 → include/jetson-inference/videoSource.h
//   * jetson-utils 独立成包的新版               → include/jetson-utils/videoSource.h
// 两版的头都是"扁平"装的（videoSource.h 里直接 #include "videoOptions.h"），
// 所以只要这一条路径对，兄弟头会由编译器在**同目录**里找到，不必再加 -I。
// __has_include 是 C++17 语法，但 gcc 5+ 在任何 -std 下都提供它（JetPack 4.x 的
// gcc 7.5 没问题）；真遇到不支持的编译器就退化成改动前的老路径。
#if defined(__has_include)
#  if __has_include("jetson-utils/videoSource.h")
#    include "jetson-utils/videoSource.h"
#  else
#    include "jetson-inference/videoSource.h"
#  endif
#else
#  include "jetson-inference/videoSource.h"
#endif

namespace
{

detectNet* g_net = nullptr;
std::string g_model_path;
std::string g_engine_path;
int g_label_count = 0;

// detectNet 用 argv 收参数，而我们是库不是可执行文件 —— 给它一份人造的。
// commandLine 只在 detectNet::Create() 里被读，所以这份 argv 只需活到那一句。
// 存进局部 vector 而不是栈上数组：commandLine 内部保存的是 char*，不拷贝。
std::string argModel;
std::string argLabels;
std::string argConfidence;

// 相机的人造 argv 同理（videoSource::Create 读的是同一批 --input-* 名字）。
// 空串 = 不传对应选项，让 GStreamer 与相机自己协商。
std::string argInputWidth;
std::string argInputHeight;
std::string argInputRate;
std::string argInputCodec;

videoSource* g_camera = nullptr;

const char* zoneOf(const std::string& class_name)
{
  // 规则与 scripts/ssd_detection_node.py 的 zone_of() 同：person 开头即人员。
  return class_name.rfind("person", 0) == 0 ? "person" : "hazard";
}

float clamp01(float value)
{
  if (!(value > 0.0f)) return 0.0f;   // NaN 也落到这里
  if (value > 1.0f) return 1.0f;
  return value;
}

// detectNet 把建好的 engine 缓存到模型旁边，但**没有提供读这个路径的接口**。
// 所以启动后在模型所在目录里找一遍。用途只有一个：台架上确认"engine 真的建成了"
// —— "模型加载成功"只说明文件读到了，而 engine 是另一步的事。
void findEngineCache(const std::string& model_path)
{
  g_engine_path.clear();
  const std::size_t slash = model_path.find_last_of("/\\");
  const std::string dir = (slash == std::string::npos) ? std::string(".")
                                                        : model_path.substr(0, slash);
  const std::string base = (slash == std::string::npos) ? model_path
                                                        : model_path.substr(slash + 1);
  // 缓存名形如 <model>.onnx.<设备>.<精度>.engine，所以按 "<base>*.engine" 找。
  const std::string pattern = dir + "/" + base + "*.engine";
  std::vector<cv::String> found;
  cv::glob(pattern, found, false);
  if (found.empty())
  {
    return;
  }
  // 多个（换过 TensorRT 版本）时取最新修改的那个。OpenCV 没有取 mtime 的
  // 接口（cv::mtime 不存在，别再写它），用 POSIX stat 直接拿。
  cv::String best;
  double best_time = -1.0;
  for (const cv::String& candidate : found)
  {
    struct stat info;
    if (stat(candidate.c_str(), &info) != 0)
    {
      continue;  // glob 命中但 stat 失败（刚被删/权限）：跳过而不是当成最新
    }
    const double stamp = static_cast<double>(info.st_mtime);
    if (stamp > best_time)
    {
      best_time = stamp;
      best = candidate;
    }
  }
  g_engine_path = best;
}

int countLabelLines(const std::string& label_path)
{
  std::FILE* handle = std::fopen(label_path.c_str(), "r");
  if (handle == nullptr)
  {
    return 0;
  }
  int count = 0;
  char line[512];
  while (std::fgets(line, sizeof(line), handle) != nullptr)
  {
    // 与 detectNet 的 LoadClassLabels 一致：去掉行尾空白，全空的行不算。
    char* cursor = line;
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == '\0' || *cursor == '\n' || *cursor == '\r')
    {
      continue;
    }
    ++count;
  }
  std::fclose(handle);
  return count;
}

}  // namespace

namespace ssd_detection
{

bool jetsonSetup(const std::string& model_path,
                 const std::string& label_path,
                 float confidence_threshold)
{
  jetsonShutdown();

  if (model_path.empty() || label_path.empty())
  {
    ROS_FATAL("ssd_detection: model_path and label_path must both be set "
              "(model='%s' labels='%s')",
              model_path.c_str(), label_path.c_str());
    return false;
  }

  g_label_count = countLabelLines(label_path);
  if (g_label_count <= 0)
  {
    ROS_ERROR("ssd_detection: cannot read label file '%s'", label_path.c_str());
    return false;
  }

  // 人造 argv。三个 blob 名**必须显式给**：detectNet 是按 --input-blob 的名字在
  // 建好的 engine 里找输入 binding 的，这个版本的默认名是 "data"，而本包契约的
  // 输入名是 input_0 ——不给就报 "failed to find requested input layer data in
  // network"（2026-10-07 实机实测：engine 已建成功、死在 binding 查找）。
  // 输出同理按 --output-cvg / --output-bbox 的名字绑。三个值就是
  // check_onnx_contract.py 守着的契约值，写死在这里、不做成节点参数。
  argModel = "--model=" + model_path;
  argLabels = "--labels=" + label_path;
  char confidence_text[32];
  std::snprintf(confidence_text, sizeof(confidence_text), "%.4f",
                static_cast<double>(confidence_threshold));
  argConfidence = std::string("--confidence=") + confidence_text;

  // 字符串字面量是静态存储期，commandLine 内部只存 char* 不拷贝，活过
  // detectNet::Create() 没问题。写法照抄旧工程 detect.cpp 那三行（那台实机
  // 验证过；下划线与连字符 commandLine 等价对待）。
  char program_name[] = "ssd_detection";
  char* argv[] = {program_name,
                  const_cast<char*>(argModel.c_str()),
                  const_cast<char*>(argLabels.c_str()),
                  const_cast<char*>(argConfidence.c_str()),
                  const_cast<char*>("--input_blob=input_0"),
                  const_cast<char*>("--output-cvg=scores"),
                  const_cast<char*>("--output_bbox=boxes"),
                  nullptr};
  commandLine cmdLine(7, argv);

  g_net = detectNet::Create(cmdLine);
  if (g_net == nullptr)
  {
    ROS_ERROR("ssd_detection: detectNet could not load '%s'. Check that the file "
              "exists and that tools/ssd_training/check_onnx_contract.py passes on it "
              "(a wrong output order or a labels.txt line-count mismatch loads fine "
              "and then reports nothing).",
              model_path.c_str());
    g_label_count = 0;
    return false;
  }

  g_model_path = model_path;
  findEngineCache(model_path);

  ROS_INFO("ssd_detection: detectNet ready.");
  ROS_INFO("  model        : %s", model_path.c_str());
  ROS_INFO("  labels       : %s (%d lines, line 0 must be BACKGROUND)",
           label_path.c_str(), g_label_count);
  ROS_INFO("  input blob   : %ux%u", g_net->GetInputWidth(), g_net->GetInputHeight());
  ROS_INFO("  network FPS  : %.1f", g_net->GetNetworkFPS());
  if (!g_engine_path.empty())
  {
    ROS_INFO("  engine cache : %s", g_engine_path.c_str());
  }
  else
  {
    ROS_WARN("  engine cache : none found next to the model. detectNet should have "
             "written one on load; if inference is about to be very slow, this is why.");
  }
  return true;
}

void jetsonShutdown()
{
  jetsonCameraClose();
  delete g_net;
  g_net = nullptr;
  g_engine_path.clear();
  g_label_count = 0;
}

bool jetsonReady()
{
  return g_net != nullptr;
}

std::string jetsonEnginePath()
{
  return g_engine_path;
}

int jetsonLabelCount()
{
  return g_label_count;
}

std::vector<JetsonDetection> jetsonInfer(const sensor_msgs::ImageConstPtr& image)
{
  std::vector<JetsonDetection> out;
  if (g_net == nullptr || !image)
  {
    return out;
  }

  // detectNet 要 RGB。cv_bridge 的 "rgb8" 已经是 RGB —— 不要再换一次，
  // 也不要用默认的 "bgr8"：preProcess 走 cudaTensorNormMeanRGB，通道序反了
  // 模型看到的就是红蓝互换的图，不报错、只是明显变差。
  cv::Mat rgb;
  try
  {
    rgb = cv_bridge::toCvCopy(image, "rgb8")->image;
  }
  catch (const cv_bridge::Exception& error)
  {
    ROS_WARN_THROTTLE(5.0, "ssd_detection: cv_bridge: %s", error.what());
    return out;
  }
  if (rgb.empty())
  {
    return out;
  }
  // cv::Mat 连续时 CV_8UC3 的内存布局与 const uchar3* 完全一致，可以直接喂；
  // 不连续（带 stride 的 ROI）则复制一份，否则 detectNet 会按紧凑布局读错行。
  if (!rgb.isContinuous())
  {
    rgb = rgb.clone();
  }
  return jetsonInferRgb8(reinterpret_cast<const uint8_t*>(rgb.data), rgb.cols,
                         rgb.rows);
}

std::vector<JetsonDetection> jetsonInferRgb8(const uint8_t* rgb, int width,
                                             int height)
{
  std::vector<JetsonDetection> out;
  if (g_net == nullptr || rgb == nullptr || width <= 0 || height <= 0)
  {
    return out;
  }

  detectNet::Detection* detections = nullptr;
  // overlayFlags = 0：框我们自己画（要按 zone 上色、还要写中文类别名），
  // 让 detectNet 再画一遍只是白花时间。
  // const_cast：detectNet 的签名是非 const（历史原因），overlayFlags=0 时它
  // 并不写这块缓冲；相机缓冲与 cv::Mat 本来就都是可写的。
  const int count = g_net->Detect(const_cast<uchar3*>(
                                      reinterpret_cast<const uchar3*>(rgb)),
                                  static_cast<uint32_t>(width),
                                  static_cast<uint32_t>(height), &detections, 0);
  if (count <= 0 || detections == nullptr)
  {
    return out;
  }

  out.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i)
  {
    const detectNet::Detection& detection = detections[i];
    // GetClassDesc 对越界 id 返回 "void"，而 postProcessSSD_ONNX 里已经
    // `if (strcmp(GetClassDesc(id), "void") == 0) continue;` 掉了，所以到这里的
    // 名字都有效；仍兜一层，因为标签与模型类数不符时症状正是"名字全错"。
    const char* description = g_net->GetClassDesc(detection.ClassID);
    if (description == nullptr || *description == '\0' ||
        std::strcmp(description, "void") == 0)
    {
      continue;
    }

    JetsonDetection item;
    item.class_name = description;
    item.zone = zoneOf(item.class_name);
    item.confidence = detection.Confidence;
    // boxes 不做 [0,1] 裁剪（见文件顶部第 2 条），贴边卡片上 Left 会略小于 0、
    // Bottom 会略大于边长。消息契约写的是归一化 0..1，下游 MissionVision 直接拿
    // 它算中心，所以在这里裁掉，不要把越界值传下去。
    item.xmin = clamp01(detection.Left / static_cast<float>(width));
    item.ymin = clamp01(detection.Top / static_cast<float>(height));
    item.xmax = clamp01(detection.Right / static_cast<float>(width));
    item.ymax = clamp01(detection.Bottom / static_cast<float>(height));
    if (item.xmax <= item.xmin || item.ymax <= item.ymin)
    {
      continue;
    }
    out.push_back(item);
  }
  return out;
}

// ---------------------------------------------------------------------------
// 相机（videoSource / GStreamer）
// ---------------------------------------------------------------------------

bool jetsonCameraOpen(const std::string& uri, int width, int height,
                      int rate, const std::string& codec)
{
  jetsonCameraClose();

  if (uri.empty())
  {
    ROS_ERROR("ssd_detection: jetsonCameraOpen called with an empty URI");
    return false;
  }

  // 人造 argv。只放调用方真的给了的选项：<=0 的宽高/帧率、空的 codec 一律
  // 不传，让 GStreamer 协商（旧版 detect.cpp 就是一个选项都不传，照样出图）。
  std::vector<std::string> args;
  if (width > 0)
  {
    argInputWidth = "--input-width=" + std::to_string(width);
    args.push_back(argInputWidth);
  }
  if (height > 0)
  {
    argInputHeight = "--input-height=" + std::to_string(height);
    args.push_back(argInputHeight);
  }
  if (rate > 0)
  {
    argInputRate = "--input-rate=" + std::to_string(rate);
    args.push_back(argInputRate);
  }
  if (!codec.empty())
  {
    argInputCodec = "--input-codec=" + codec;
    args.push_back(argInputCodec);
  }

  std::vector<char*> argv;
  char program_name[] = "ssd_detection_camera";
  argv.push_back(program_name);
  for (const std::string& arg : args)
  {
    argv.push_back(const_cast<char*>(arg.c_str()));
  }
  argv.push_back(nullptr);

  commandLine cmdLine(static_cast<int>(args.size()) + 1, argv.data());
  g_camera = videoSource::Create(uri.c_str(), cmdLine);
  if (g_camera == nullptr)
  {
    ROS_ERROR("ssd_detection: videoSource could not open '%s'. On the Nano, "
              "check the device exists (v4l2-ctl --list-devices) and that "
              "nothing else holds it.",
              uri.c_str());
    return false;
  }
  if (g_camera->GetWidth() == 0 || g_camera->GetHeight() == 0)
  {
    ROS_ERROR("ssd_detection: camera '%s' opened but reports a %ux%u stream; "
              "GStreamer negotiated nothing. Check --input-codec against what "
              "the camera actually offers.",
              uri.c_str(), g_camera->GetWidth(), g_camera->GetHeight());
    jetsonCameraClose();
    return false;
  }
  ROS_INFO("ssd_detection: camera '%s' streaming at %ux%u (requested %dx%d, "
           "codec '%s')",
           uri.c_str(), g_camera->GetWidth(), g_camera->GetHeight(), width,
           height, codec.empty() ? "<auto>" : codec.c_str());
  return true;
}

void jetsonCameraClose()
{
  delete g_camera;
  g_camera = nullptr;
}

bool jetsonCameraCapture(cv::Mat& rgb, int timeout_ms)
{
  if (g_camera == nullptr)
  {
    return false;
  }

  uchar3* image = nullptr;
  if (!g_camera->Capture(&image, static_cast<uint64_t>(timeout_ms)))
  {
    return false;
  }

  // 浅拷贝视图：指向相机内部缓冲区，不拷贝像素。见头文件里的寿命规则 ——
  // 下一次 Capture 之前必须用完（推理/画框/编码都在同一个循环迭代里）。
  rgb = cv::Mat(static_cast<int>(g_camera->GetHeight()),
                static_cast<int>(g_camera->GetWidth()), CV_8UC3,
                static_cast<void*>(image));
  return true;
}

int jetsonCameraWidth()
{
  return g_camera == nullptr ? 0 : static_cast<int>(g_camera->GetWidth());
}

int jetsonCameraHeight()
{
  return g_camera == nullptr ? 0 : static_cast<int>(g_camera->GetHeight());
}

bool jetsonCameraIsStreaming()
{
  return g_camera != nullptr && g_camera->IsStreaming();
}

}  // namespace ssd_detection
