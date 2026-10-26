/***************************************************************************************************************************
 *
 * Author: expload
 * Time: 2026.10
 * Description: YOLOv5 / YOLOv8（raw head）检测，两个后端：
 *   - RKNN（RK3399Pro，板上）：detect_mode/26/best.rknn，NPU 跑卷积；
 *   - OpenCV DNN（x86 / Gazebo 仿真）：同一个 raw-head ONNX，纯 CPU。
 * 由 CMake 的 USE_RKNN / USE_ONNX 决定编哪个，运行时可用 ~backend 覆盖。
 *
 * 模型：输入 1x3xSxS（0..1，RGB），输出 p3/p4/p5 三个 raw head。
 *       交回四类模型为 [1,3,H,W,9]，应用层转换为 [1,27,H,W]；
 *       亦支持 YOLOv5 的 [1,3*(5+nc),H,W] 输出。
 *       每格每 anchor 是 tx,ty,tw,th,obj,cls…（都是 logits）；YOLOv8 则是 DFL。
 *       解码（anchor / sigmoid / DFL）在 inferYolo() 里做，图里没有 NMS。
 *
 * 后处理（阈值 + 逐类 NMS + letterbox 反变换）在 CPU 上做，RKNN 只跑卷积，
 * RK3399Pro 的 NPU 兼容性最好；两个后端共用同一份解码。
 ***************************************************************************************************************************/
#ifndef NAVIGATION_2D_DETECT_H
#define NAVIGATION_2D_DETECT_H

#include <atomic>
#include <chrono>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include "yolo_tensor_layout.h"
#include "latest_result_cache.h"
#include "detection_freshness.h"

#ifdef USE_RKNN
#include "rknn_api.h"
#endif
#ifdef USE_ONNX
#include <opencv2/dnn.hpp>
#endif

class Detect
{
public:
  struct Detection
  {
    std::string label;
    float confidence = 0.0f;
    cv::Rect2f box;        // 像素坐标，源图分辨率
    cv::Point2f center;    // 像素坐标
    int class_id = -1;
  };

  Detect();
  Detect(int argc, char **argv);
  ~Detect();

  bool ready() const { return ready_.load(); }
  bool hasFrame() const;
  bool lastInferenceSucceeded() const;
  bool inferenceReady() const;
  bool startAsync();
  void stopAsync();
  // Each consumer passes its previous sequence; empty detections are also a
  // completed new result. Polling never waits for NPU inference.
  bool pollAsync(std::vector<Detection> &detections, uint64_t &sequence) const;

  /// 模型 labels.txt 里是否有这个类名（用于校验运行前输入的危险品类别）。
  bool hasLabel(const std::string &label) const;

  /// 对最新一帧跑一次推理，返回阈值以上、做过 NMS 的全部目标。
  std::vector<Detection> infer();

  /// 读一次参数（构造函数已自动调用；保留给单测/调试）。
  void loadParams();

  cv::Size frameSize() const;

  /// 像素 -> 机体系（x 前，y 左，单位米）。
  /// height_m 是相机到目标平面的高度；camera_yaw_deg 把相机图像的上方向旋到机头方向。
  bool pixelToBody(const cv::Point2f &pixel, double height_m, cv::Point2f &body) const;

  /// 在回传画面上画框。同一个 tag 的新框会替换旧框，hold_sec 秒后自动消失。
  void annotate(const Detection &det, const cv::Scalar &color, const std::string &tag,
                double hold_sec = 3.0);
  void clearAnnotations();

private:
  /// 一个网络输出张量（CHW，已拉平）。
  typedef navigation_yolo::Head Tensor;
  struct AsyncResult
  {
    std::vector<Detection> detections;
    ros::Time source_stamp;
    std::chrono::steady_clock::time_point source_received, completed;
    double source_age_at_receipt = 0.0;
    bool success = false;
  };
  int expectedChannels() const;
  void asyncLoop();
  std::vector<Detection> inferImpl(AsyncResult *completed_result);

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void publishCallback(const ros::TimerEvent &event);
  bool initRknn(const std::string &model_file);
  void releaseRknn();
  /// x86/仿真后端：OpenCV DNN 读同一个 raw-head ONNX。
  bool initOnnx(const std::string &model_file);
  void releaseOnnx();
  /// YOLO（raw head）：网络一次吐 3 个张量，anchor/obj/class 解码都在这里做。
  std::vector<Detection> inferYolo(const cv::Mat &frame);
  bool runYoloInference(const cv::Mat &canvas, std::vector<Tensor> &tensors);

  struct Annotation
  {
    Detection det;
    cv::Scalar color;
    std::string tag;
    ros::Time expires;
  };

  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;
  ros::Timer publish_timer_;

  mutable std::mutex mutex_;
  std::mutex inference_mutex_;
  std::mutex worker_mutex_;
  std::thread worker_;
  std::atomic<bool> async_running_{false};
  navigation_yolo::LatestResultCache<AsyncResult> async_results_;
  cv::Mat frame_;              // bgr8，最新一帧
  ros::Time frame_stamp_;
  ros::Time inference_source_stamp_;
  std::chrono::steady_clock::time_point frame_received_, inference_received_, inference_source_received_;
  double frame_source_age_at_receipt_ = 0.0, inference_source_age_at_receipt_ = 0.0;
  bool last_inference_ok_ = false;
  uint64_t frame_generation_ = 0, last_inferred_generation_ = 0;
  double frame_timeout_ = 1.0;
  double inference_timeout_ = 3.0;
  std::string frame_id_ = "camera";
  std::vector<Annotation> annotations_;

  std::vector<std::string> labels_;
  std::string input_topic_;
  std::string model_path_, model_file_, labels_file_;
  /// 推理后端：auto（编进哪个用哪个）/ rknn / onnx / none
  std::string backend_ = "auto";
  std::string onnx_file_ = "best.onnx";
  double confidence_threshold_ = 0.5;
  double nms_iou_ = 0.45;
  double camera_hfov_deg_ = 54.0;
  double camera_fx_ = 0.0, camera_fy_ = 0.0;
  double camera_yaw_deg_ = 0.0;
  double publish_scale_ = 0.5;
  double publish_rate_ = 8.0;
  int top_k_ = 100;

  std::atomic<bool> ready_{false};
  int input_size_ = 416;
  int yolo_reg_max_ = 16;   // DFL 的 bin 数（只有 v8-dfl 头用到）
  /// YOLO 头风格：
  ///   v5-interleaved —— yolov5 <= v6.0，单卷积，通道按 anchor 交织 [t,obj,cls…]
  ///   v5-split       —— yolov5 v6.2+，[box×na | cls×na]（无 obj）
  ///   v8-dfl         —— YOLOv8/11，box 是 DFL 的 4*reg_max
  std::string yolo_style_ = "v5-interleaved";
  std::string yolo_anchors_csv_, yolo_strides_csv_;
  std::vector<float> anchors_;   // [level][anchor][w,h]，像素
  std::vector<int> strides_;

#ifdef USE_RKNN
  rknn_context ctx_ = 0;
  rknn_input_output_num io_num_;
  rknn_tensor_attr input_attr_;
  static const int kMaxOutputs = 8;
  rknn_tensor_attr output_attr_[kMaxOutputs];
  int n_outputs_ = 0;
#endif
#ifdef USE_ONNX
  cv::dnn::Net net_;
  std::vector<std::string> out_names_;
#endif
};

#endif  // NAVIGATION_2D_DETECT_H
