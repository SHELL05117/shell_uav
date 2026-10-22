#pragma once

// jetson-inference detectNet 的对外接口。
//
// 与被删掉的 ssd_trt_infer.h 同一套纪律：**只暴露自由函数、只用 ROS 类型**。
// detectNet / cudaTensor / nvinfer 这些头文件只出现在 jetson_detector.cpp 里，
// ROS 那一侧（ssd_detection_node.cpp）一个都看不到。这样" Jetson 上编不过"
// 与"ROS 逻辑写错"是两件能分开看的事，也让本文件可以被 PC 上的语法检查读懂。
//
// 为什么是 detectNet 而不是自己读 .plan
// ------------------------------------
// .plan / .uff 那条链要三样东西：graphsurgeon 往图里注入 GridAnchor_TRT /
// NMS_TRT / FlattenConcat_TRT 三个**自定义插件**、trtexec、以及构建方与运行期
// 持有**字节格式一致**的插件实现 —— 最后这条无法验证，"engine 建出来了"并不
// 证明我们的节点读得懂它。
//
// detectNet 这条链没有插件：模型停在 ONNX，首帧由 detectNet 自己解析并建
// TensorRT engine，缓存在模型旁边（TRT 版本变了会自动重建）。代价是模型的
// 输出约定必须符合 detectNet 的契约，而那份契约可以在本机上静态核对：
//     tools/ssd_training/check_onnx_contract.py --onnx X.onnx --labels X_labels.txt
//
// 三条契约里最容易中的招（**这一条本文件挡不住，只能靠上面那个脚本提前查**）：
// detectNet 从**张量**里读类数（mNumClasses = scores 的最后一维），从
// **labels.txt** 里读类名，而这条分支**从不核对两者**。标签少一行，类名整体
// 错位，越界 id 被 GetClassDesc() 判成 "void" 后逐条丢弃 —— engine 正常、
// 帧率正常、永远零检出、不打印任何东西。

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>

namespace ssd_detection
{

struct JetsonDetection
{
  std::string class_name;
  std::string zone;      // "hazard" 或 "person"，规则见下方说明
  float confidence;
  float xmin;            // 归一化 0..1，已裁剪到 [0,1]；顺序与 VisionDetection.msg 一致
  float ymin;
  float xmax;
  float ymax;
};

// zone 的规则：类别名以 "person" 开头就是 person，其余都是 hazard。
// 与 scripts/ssd_detection_node.py 的 zone_of() 是**同一条规则的两份拷贝**，
// 抄自 labels.py 的意图。改类别名时两处都要改 —— 这是已知的坑（README 坑清单），
// 所以规则写成"看前缀"而不是抄一张 10 行的表：抄表的话改一次名字就有一处会忘。

// 加载 ONNX 模型与 label 文件。返回 false 表示没就绪（文件不存在、或
// detectNet 建引擎失败），此时 jetsonReady() 也是 false，jetsonInfer() 返回空。
//
// label 文件的**行 0 必须是 background**，且**行数必须等于模型 scores 的类维**
// （10 类任务模型 ⇒ 11 行）。这两条 detectNet 自己不核对，见本文件顶部。
//
// confidence_threshold 直接决定 detectNet 的 mConfidenceThreshold，改它需要
// 重新 jetsonSetup()，也就是要重启节点 —— 飞行中不能调。
bool jetsonSetup(const std::string& model_path,
                 const std::string& label_path,
                 float confidence_threshold);

void jetsonShutdown();
bool jetsonReady();

// detectNet 首帧会把建好的 engine 缓存到模型旁边（文件名形如
// <model>.onnx.<设备>.<精度>.engine）。把这个路径报出来是给台架看的：它证明
// engine 真的建成了，而"模型加载成功"只说明文件读到了。空字符串 = 还没建。
std::string jetsonEnginePath();

// label 文件读到的类数（含背景）。与模型对不上时**这里不会失败**，所以节点
// 只把它打出来 —— 真正的判据是 check_onnx_contract.py。
int jetsonLabelCount();

// image 必须是 sensor_msgs/Image（内部按 rgb8 取）。返回的框已按 detectNet 的
// 阈值过滤，并裁剪到 [0,1]。
std::vector<JetsonDetection> jetsonInfer(const sensor_msgs::ImageConstPtr& image);

// 同一个推理的**裸指针入口**：RGB8（uchar3 布局，3 字节/像素，R,G,B 顺序）。
// 相机模式下没有 ROS 消息可包 —— videoSource 给的就是映射到 CPU 的 NVMM
// 零拷贝缓冲，包成 Image 再解一次纯属浪费。两条路最终都走 detectNet 的
// 同一个 Detect()，所以"台架上用消息喂"与"飞行中用相机喂"看到的是同一个模型。
std::vector<JetsonDetection> jetsonInferRgb8(const uint8_t* rgb, int width,
                                             int height);

// ---------------------------------------------------------------------------
// 相机（jetson-inference videoSource，全程 GStreamer）。
//
// 为什么不用 usb_cam 节点 + /camera/image_raw：那样一帧图要进 ROS 一次、
// 被两个订阅者各 cv_bridge 一次，MJPEG 解码在 CPU 上做，而这里只需要**一个**
// 回传画面。videoSource 的 v4l2:// 通路在 Jetson 上是
//     v4l2src ! image/jpeg caps ! jpegparse ! nvv4l2decoder mjpeg=1
//            ! nvvidconv ! video/x-raw(memory:NVMM) ! appsink
// —— MJPEG 走硬件解码，终点是 NVMM 显存，Capture() 交回来的指针是映射到 CPU
// 的同一块内存（零拷贝），可以直接喂 detectNet，也可以直接包成 cv::Mat 画框。
//
// URI 形如 `v4l2:///dev/video0`（USB 相机）或 `csi://0`（MIPI CSI 相机）。
// 空 URI = 不建相机（节点退回订阅模式）。
// width/height/rate 是**请求值**，相机不支持时由 GStreamer 协商；启动后从
// jetsonCameraWidth()/Height() 读实际值，别假设请求值生效了。
bool jetsonCameraOpen(const std::string& uri, int width, int height,
                      int rate, const std::string& codec);
void jetsonCameraClose();

// 抓一帧，阻塞最多 timeout_ms 毫秒。成功时 rgb 被赋值成一张 CV_8UC3 的
// **浅拷贝视图**，指向相机内部缓冲区 —— 下一次调用本函数它即失效，
// 所以必须在下一次抓帧前用完（推理、画框、编码都在同一个循环迭代里完成，
// 见 ssd_detection_node.cpp 的主循环）。
bool jetsonCameraCapture(cv::Mat& rgb, int timeout_ms);

int jetsonCameraWidth();
int jetsonCameraHeight();

// 相机还在出图吗。GStreamer 的 ERROR/EOS（设备被拔、驱动复位）会把流关掉，
// 那时候 Capture() 只会一直超时 —— 主循环靠这个区分"等一帧"和"相机死了"。
bool jetsonCameraIsStreaming();

}  // namespace ssd_detection
