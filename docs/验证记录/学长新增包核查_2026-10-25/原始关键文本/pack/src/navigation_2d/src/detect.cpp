/***************************************************************************************************************************
 *
 * Author: expload
 * Time: 2026.10
 * Description: RKNN (RK3399Pro) 版 detect。见 detect.h 顶部的说明。
 ***************************************************************************************************************************/
#include "detect.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace
{
/// "10,13,16,30" -> {10,13,16,30}
void parseCsvFloats(const std::string &csv, std::vector<float> &out)
{
  out.clear();
  std::stringstream stream(csv);
  std::string item;
  while (std::getline(stream, item, ','))
  {
    if (item.empty())
      continue;
    out.push_back(static_cast<float>(std::atof(item.c_str())));
  }
}

void parseCsvInts(const std::string &csv, std::vector<int> &out)
{
  out.clear();
  std::stringstream stream(csv);
  std::string item;
  while (std::getline(stream, item, ','))
  {
    if (item.empty())
      continue;
    out.push_back(std::atoi(item.c_str()));
  }
}

float iou(const cv::Rect2f &a, const cv::Rect2f &b)
{
  const float x1 = std::max(a.x, b.x);
  const float y1 = std::max(a.y, b.y);
  const float x2 = std::min(a.x + a.width, b.x + b.width);
  const float y2 = std::min(a.y + a.height, b.y + b.height);
  const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
  const float uni = a.width * a.height + b.width * b.height - inter;
  return uni > 0.0f ? inter / uni : 0.0f;
}

// "auto" 就是"编进哪个用哪个"，由 CMake 的 -DUSE_RKNN / -DUSE_ONNX 决定。
const char *defaultBackend()
{
#ifdef USE_RKNN
  return "rknn";
#elif defined(USE_ONNX)
  return "onnx";
#else
  return "none";
#endif
}
}  // namespace

Detect::Detect()
{
  loadParams();
}

Detect::Detect(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  loadParams();
}

Detect::~Detect()
{
  releaseRknn();
  releaseOnnx();
}

void Detect::loadParams()
{
  nh_ = ros::NodeHandle("~");

  nh_.param<std::string>("input_image_topic", input_topic_, "/camera/image_raw");
  nh_.param<std::string>("model_path", model_path_, "");
  nh_.param<std::string>("model_file", model_file_, "gc-yolo-raw-416.rknn");
  nh_.param<std::string>("labels_file", labels_file_, "labels.txt");
  nh_.param<std::string>("backend", backend_, "auto");
  nh_.param<std::string>("onnx_file", onnx_file_, "gc-yolo-raw-416.onnx");
  nh_.param<int>("yolo_reg_max", yolo_reg_max_, 16);
  nh_.param<std::string>("yolo_style", yolo_style_, "v5-interleaved");
  nh_.param<std::string>("yolo_anchors", yolo_anchors_csv_, "");
  nh_.param<std::string>("yolo_strides", yolo_strides_csv_, "8,16,32");
  parseCsvFloats(yolo_anchors_csv_, anchors_);
  parseCsvInts(yolo_strides_csv_, strides_);
  if (yolo_style_ != "v5-interleaved" && yolo_style_ != "v5-split" && yolo_style_ != "v8-dfl")
  {
    ROS_WARN("Detect: unknown ~yolo_style '%s' (v5-interleaved|v5-split|v8-dfl); "
             "falling back to v5-interleaved", yolo_style_.c_str());
    yolo_style_ = "v5-interleaved";
  }
  if (yolo_style_ != "v8-dfl" && anchors_.size() < 18)
    ROS_WARN("Detect: ~yolo_anchors has %zu value(s), yolov5 needs 18 (3 levels x 3 "
             "anchors x 2) -- paste the yolo_anchors line printed by "
             "tools/export_yolo_raw.py", anchors_.size());
  int input_size_param = 0;
  nh_.param<int>("input_size", input_size_param, 0);
  if (input_size_param > 0)
    input_size_ = input_size_param;
  nh_.param<double>("confidence_threshold", confidence_threshold_, 0.5);
  nh_.param<double>("nms_iou", nms_iou_, 0.45);
  nh_.param<double>("camera_hfov_deg", camera_hfov_deg_, 54.0);
  nh_.param<double>("camera_fx", camera_fx_, 0.0);
  nh_.param<double>("camera_fy", camera_fy_, 0.0);
  nh_.param<double>("camera_yaw_deg", camera_yaw_deg_, 0.0);
  nh_.param<double>("publish_scale", publish_scale_, 0.5);
  nh_.param<double>("publish_rate", publish_rate_, 8.0);

  // labels.txt: 一行一类，顺序 = 训练时的 class id（**没有 BACKGROUND**，
  // 因为 raw head 的类别 logits 就是 nc 个）。
  labels_.clear();
  const std::string labels_path = model_path_.empty() ? labels_file_
                                                       : (model_path_ + "/" + labels_file_);
  std::ifstream label_stream(labels_path.c_str());
  if (label_stream.is_open())
  {
    std::string line;
    while (std::getline(label_stream, line))
    {
      while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
        line.pop_back();
      if (!line.empty())
        labels_.push_back(line);
    }
  }
  if (labels_.empty())
  {
    ROS_ERROR("Detect: cannot read labels from %s; the node will not detect anything",
              labels_path.c_str());
  }
  else
  {
    ROS_INFO("Detect: %zu labels from %s", labels_.size(), labels_path.c_str());
  }

  image_pub_ = nh_.advertise<sensor_msgs::Image>("/detect_result", 1);
  image_sub_ = nh_.subscribe(input_topic_, 1, &Detect::imageCallback, this,
                             ros::TransportHints().tcpNoDelay());
  publish_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)),
                                   &Detect::publishCallback, this);

  const std::string model_path = model_path_.empty() ? model_file_ : (model_path_ + "/" + model_file_);
  const std::string onnx_path = model_path_.empty() ? onnx_file_ : (model_path_ + "/" + onnx_file_);
  if (labels_.empty())
    return;

  // 后端：auto = 编译进去的那个（板上是 rknn，x86/仿真上是 onnx）。显式指定一个
  // 没编进来的后端时，它自己的 stub 会打警告并保持 ready_=false，任务照常按固定
  // 投放点飞，不会崩。
  if (backend_ == "auto")
    backend_ = defaultBackend();
  ROS_INFO("Detect: backend=%s (model_path=%s)", backend_.c_str(),
           model_path_.empty() ? "(none)" : model_path_.c_str());
  if (backend_ == "rknn")
    initRknn(model_path);
  else if (backend_ == "onnx")
    initOnnx(onnx_path);
  else if (backend_ == "none")
    ROS_WARN("Detect: backend=none -- no inference; waypoints still fly, drops fall back to "
             "the fixed positions");
  else
    ROS_ERROR("Detect: unknown ~backend '%s' (auto|rknn|onnx|none); no inference",
              backend_.c_str());
}

bool Detect::hasFrame() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !frame_.empty();
}

bool Detect::hasLabel(const std::string &label) const
{
  return std::find(labels_.begin(), labels_.end(), label) != labels_.end();
}

cv::Size Detect::frameSize() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_.empty() ? cv::Size(0, 0) : frame_.size();
}

void Detect::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // 统一成 bgr8；usb_cam 一般直接给 bgr8，Gazebo 相机给 rgb8，cv_bridge 会转。
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception &e)
  {
    ROS_ERROR_THROTTLE(2.0, "Detect: cv_bridge: %s", e.what());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  frame_ = cv_ptr->image.clone();
  frame_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  frame_id_ = msg->header.frame_id.empty() ? "camera" : msg->header.frame_id;
}

#ifdef USE_RKNN
bool Detect::initRknn(const std::string &model_file)
{
  std::ifstream stream(model_file.c_str(), std::ios::binary | std::ios::ate);
  if (!stream.is_open())
  {
    ROS_ERROR("Detect: cannot open RKNN model %s (run tools/convert_onnx2rknn.py first, "
              "or set ~model_path/~model_file); falling back to fixed drop positions",
              model_file.c_str());
    return false;
  }
  const std::streamsize size = stream.tellg();
  stream.seekg(0, std::ios::beg);
  std::vector<char> data(static_cast<size_t>(size));
  if (!stream.read(data.data(), size))
  {
    ROS_ERROR("Detect: short read on %s", model_file.c_str());
    return false;
  }

  int ret = rknn_init(&ctx_, data.data(), static_cast<uint32_t>(data.size()), 0);
  if (ret < 0)
  {
    ROS_ERROR("Detect: rknn_init failed (%d) on %s. Check that the model was built for "
              "rk3399pro and that the board NPU driver matches rknn-toolkit 1.7.5",
              ret, model_file.c_str());
    ctx_ = 0;
    return false;
  }

  ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
  if (ret != RKNN_SUCC)
  {
    ROS_ERROR("Detect: rknn_query(IN_OUT_NUM) failed: %d", ret);
    return false;
  }
  ROS_INFO("Detect: RKNN model %s: %u input(s), %u output(s)", model_file.c_str(),
           io_num_.n_input, io_num_.n_output);
  if (io_num_.n_input != 1)
  {
    ROS_ERROR("Detect: expected 1 input, got %u", io_num_.n_input);
    return false;
  }
  if (io_num_.n_output < 3)
  {
    ROS_ERROR("Detect: raw-head 模型应该有 3 个输出 (p3,p4,p5)，这个只有 %u 个",
              io_num_.n_output);
    return false;
  }
  n_outputs_ = std::min<int>(static_cast<int>(io_num_.n_output), kMaxOutputs);

  memset(&input_attr_, 0, sizeof(input_attr_));
  input_attr_.index = 0;
  ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &input_attr_, sizeof(input_attr_));
  if (ret != RKNN_SUCC)
  {
    ROS_ERROR("Detect: rknn_query(INPUT_ATTR) failed: %d", ret);
    return false;
  }
  ROS_INFO("Detect: input '%s' dims=[%u,%u,%u,%u] fmt=%d type=%d", input_attr_.name,
           input_attr_.dims[0], input_attr_.dims[1], input_attr_.dims[2], input_attr_.dims[3],
           input_attr_.fmt, input_attr_.type);

  // 正方形输入：NCHW 取 dims[2]，NHWC 取 dims[1]
  if (input_attr_.dims[2] == input_attr_.dims[3] && input_attr_.dims[2] > 1)
    input_size_ = static_cast<int>(input_attr_.dims[2]);
  else if (input_attr_.dims[1] == input_attr_.dims[2] && input_attr_.dims[1] > 1)
    input_size_ = static_cast<int>(input_attr_.dims[1]);
  else
  {
    ROS_ERROR("Detect: unsupported input shape [%u,%u,%u,%u] (want 1x3xSxS)",
              input_attr_.dims[0], input_attr_.dims[1], input_attr_.dims[2],
              input_attr_.dims[3]);
    return false;
  }
  ROS_INFO("Detect: input_size=%d", input_size_);

  for (int i = 0; i < n_outputs_; ++i)
  {
    memset(&output_attr_[i], 0, sizeof(output_attr_[i]));
    output_attr_[i].index = static_cast<uint32_t>(i);
    ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &output_attr_[i], sizeof(output_attr_[i]));
    if (ret != RKNN_SUCC)
    {
      ROS_ERROR("Detect: rknn_query(OUTPUT_ATTR %d) failed: %d", i, ret);
      return false;
    }
    ROS_INFO("Detect: output[%d] '%s' dims=[%u,%u,%u,%u] fmt=%d type=%d n_elems=%u", i,
             output_attr_[i].name, output_attr_[i].dims[0], output_attr_[i].dims[1],
             output_attr_[i].dims[2], output_attr_[i].dims[3], output_attr_[i].fmt,
             output_attr_[i].type, output_attr_[i].n_elems);
  }

  ready_ = true;
  ROS_INFO("Detect: RKNN ready (%s)", model_file.c_str());
  return true;
}

void Detect::releaseRknn()
{
  if (ctx_ != 0)
  {
    rknn_destroy(ctx_);
    ctx_ = 0;
  }
  ready_ = false;
}
#else
bool Detect::initRknn(const std::string &model_file)
{
  (void)model_file;
  ROS_WARN("Detect: built without USE_RKNN; no inference. Waypoints still fly, drops fall "
           "back to the fixed positions.");
  ready_ = false;
  return false;
}

void Detect::releaseRknn() { ready_ = false; }
#endif

// ---------------------------------------------------------------------------
// OpenCV DNN 后端（x86 / 仿真）：读同一个 raw-head ONNX。
//   input  images  [1,3,S,S] float32 0..1
//   output p3/p4/p5（YOLOv5 每层 [1, 3*(5+nc), H, W]，通道是 logits）
// 图里只有 Conv/LeakyReLU/MaxPool/Concat/Upsample，解码在 inferYolo() 里做。
//
// Exp shim 保留：只有老式的 SSD 解码尾才需要它，现在用不到了；但 OpenCV < 4.5
// 万一碰到别的模型带 Exp，注册一下也不亏（4.5+ 不注册，用自带的）。
// ---------------------------------------------------------------------------
#ifdef USE_ONNX

#if (CV_VERSION_MAJOR < 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR < 5)
namespace
{
class ExpLayer : public cv::dnn::Layer
{
public:
  explicit ExpLayer(const cv::dnn::LayerParams &params) : cv::dnn::Layer(params) {}

  static cv::Ptr<cv::dnn::Layer> create(cv::dnn::LayerParams &params)
  {
    return cv::makePtr<ExpLayer>(params);
  }

  bool getMemoryShapes(const std::vector<std::vector<int> > &inputs, const int requiredOutputs,
                       std::vector<std::vector<int> > &outputs,
                       std::vector<std::vector<int> > &internals) const CV_OVERRIDE
  {
    (void)internals;
    outputs.assign(std::max(requiredOutputs, 1), inputs[0]);
    return true;
  }

  void forward(cv::InputArrayOfArrays inputs_arr, cv::OutputArrayOfArrays outputs_arr,
               cv::OutputArrayOfArrays) CV_OVERRIDE
  {
    std::vector<cv::Mat> inputs, outputs;
    inputs_arr.getMatVector(inputs);
    outputs_arr.getMatVector(outputs);
    cv::exp(inputs[0], outputs[0]);
  }
};

// 必须在 readNetFromONNX() **之前**调用：importer 解析图的时候就要实例化每一层。
void registerMissingDnnLayers()
{
  static bool done = false;
  if (done)
    return;
  done = true;
  try
  {
    cv::dnn::LayerFactory::registerLayer("Exp", ExpLayer::create);
    ROS_INFO("Detect: OpenCV %s has no Exp layer; registered a minimal one "
             "(YOLO 的图里没有 Exp，注册了也不影响)", CV_VERSION);
  }
  catch (const cv::Exception &e)
  {
    ROS_WARN("Detect: could not register the Exp layer: %s", e.what());
  }
}
}  // namespace
#else
namespace
{
void registerMissingDnnLayers() {}
}  // namespace
#endif

bool Detect::initOnnx(const std::string &model_file)
{
  registerMissingDnnLayers();
  try
  {
    net_ = cv::dnn::readNetFromONNX(model_file);
  }
  catch (const cv::Exception &e)
  {
    ROS_ERROR("Detect: cannot build ONNX model %s with cv::dnn %s:\n  %s\n"
              "  raw-head 的图里只有 Conv/LeakyReLU/MaxPool/Concat/Upsample，正常都能读。"
              "如果报 \"Can't create layer ... of type X\"，就是这个 OpenCV 缺该层"
              "（Exp 已经补过 shim，别的名字要照同样办法补，或者用 OpenCV >= 4.5）。",
              model_file.c_str(), CV_VERSION, e.what());
    return false;
  }
  if (net_.empty())
  {
    ROS_ERROR("Detect: ONNX model %s produced an empty network", model_file.c_str());
    return false;
  }
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

  out_names_ = net_.getUnconnectedOutLayersNames();
  for (size_t i = 0; i < out_names_.size(); ++i)
    ROS_INFO("Detect: ONNX output[%zu] '%s'", i, out_names_[i].c_str());
  if (out_names_.size() < 3)
  {
    ROS_ERROR("Detect: raw-head 模型应该有 3 个输出 (p3,p4,p5)，这个 onnx 只有 %zu 个",
              out_names_.size());
    return false;
  }

  ready_ = true;
  ROS_INFO("Detect: ONNX ready via OpenCV DNN %s (%s, input %dx%d)", CV_VERSION,
           model_file.c_str(), input_size_, input_size_);
  return true;
}

void Detect::releaseOnnx()
{
  net_ = cv::dnn::Net();
  ready_ = false;
}
#else
bool Detect::initOnnx(const std::string &model_file)
{
  (void)model_file;
  ROS_WARN("Detect: built without USE_ONNX (OpenCV DNN backend); no inference. Rebuild with "
           "-DUSE_ONNX=ON, or run on the board with -DUSE_RKNN=ON.");
  ready_ = false;
  return false;
}

void Detect::releaseOnnx() { ready_ = false; }
#endif

// ---------------------------------------------------------------------------
// 推理后端
//
// 板上：RKNN（librknn_runtime，int8 量化，输入 uint8 RGB，mean/std 在 rknn.config
//       里做；三个 raw head 输出 want_float=1 拿回来）。
// x86 仿真：OpenCV DNN 读同一个 raw-head ONNX（输入 0..1 float）。
// 两个后端拿到的东西一模一样，解码/阈值/NMS/letterbox 全在 inferYolo() 里共用。
// ---------------------------------------------------------------------------
std::vector<Detect::Detection> Detect::infer()
{
  std::vector<Detection> results;
#if defined(USE_RKNN) || defined(USE_ONNX)
  cv::Mat frame;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_ || frame_.empty())
      return results;
    frame = frame_.clone();
  }
  return inferYolo(frame);
#else
  return results;
#endif
}

std::vector<Detect::Detection> Detect::inferYolo(const cv::Mat &frame)
{
  std::vector<Detection> results;
  const int nc = static_cast<int>(labels_.size());
  const int reg_max = std::max(1, yolo_reg_max_);
  const int box_channels = 4 * reg_max;
  const int channels = box_channels + nc;
  if (nc < 1 || frame.empty())
    return results;

  const double scale = std::min(input_size_ / static_cast<double>(frame.cols),
                                input_size_ / static_cast<double>(frame.rows));
  const int new_w = std::max(1, static_cast<int>(std::lround(frame.cols * scale)));
  const int new_h = std::max(1, static_cast<int>(std::lround(frame.rows * scale)));
  const int pad_x = (input_size_ - new_w) / 2;
  const int pad_y = (input_size_ - new_h) / 2;

  cv::Mat canvas(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(new_w, new_h));
  resized.copyTo(canvas(cv::Rect(pad_x, pad_y, new_w, new_h)));

  std::vector<Tensor> tensors;
  if (!runYoloInference(canvas, tensors))
    return results;

  std::vector<Detection> candidates;
  candidates.reserve(256);
  const bool v8 = (yolo_style_ == "v8-dfl");
  const int na = 3;                          // yolov5 每层 3 个 anchor
  const int v5_interleaved = na * (5 + nc);  // [tx,ty,tw,th,obj,cls...] × na
  const int v5_split = na * 4 + na * nc;     // [box×na | cls×na]
  for (size_t k = 0; k < tensors.size(); ++k)
  {
    const Tensor &t = tensors[k];
    if (t.height < 1 || t.width < 1)
      continue;
    const int stride = input_size_ / t.height;
    if (stride < 1)
      continue;
    const size_t plane = static_cast<size_t>(t.height) * t.width;

    // 这一层对应的 stride 序号（决定用哪组 anchor）
    int level = -1;
    for (size_t i = 0; i < strides_.size(); ++i)
      if (strides_[i] == stride)
        level = static_cast<int>(i);

    if (v8)
    {
      if (t.channels != channels)
        continue;
      for (int y = 0; y < t.height; ++y)
      {
        for (int x = 0; x < t.width; ++x)
        {
          const size_t pixel = static_cast<size_t>(y) * t.width + x;
          float best_score = -1.0f;
          int best_class = -1;
          for (int c = 0; c < nc; ++c)
          {
            const float logit = t.data[(box_channels + c) * plane + pixel];
            const float score = 1.0f / (1.0f + std::exp(-logit));
            if (score > best_score)
            {
              best_score = score;
              best_class = c;
            }
          }
          if (best_class < 0 || best_score < confidence_threshold_)
            continue;

          float distance[4] = {0.0f, 0.0f, 0.0f, 0.0f};
          for (int side = 0; side < 4; ++side)
          {
            const size_t base = static_cast<size_t>(side) * reg_max * plane;
            float max_logit = -1e30f;
            for (int b = 0; b < reg_max; ++b)
              max_logit = std::max(max_logit, t.data[base + b * plane + pixel]);
            float sum = 0.0f;
            float acc = 0.0f;
            for (int b = 0; b < reg_max; ++b)
            {
              const float e = std::exp(t.data[base + b * plane + pixel] - max_logit);
              sum += e;
              acc += e * static_cast<float>(b);
            }
            distance[side] = (sum > 0.0f) ? acc / sum : 0.0f;
          }

          const float cx = (x + 0.5f) * stride;
          const float cy = (y + 0.5f) * stride;
          const float x1 = cx - distance[0] * stride;
          const float y1 = cy - distance[1] * stride;
          const float x2 = cx + distance[2] * stride;
          const float y2 = cy + distance[3] * stride;
          if (x2 - x1 < 1.0f || y2 - y1 < 1.0f)
            continue;

          Detection det;
          det.label = labels_[best_class];
          det.class_id = best_class;
          det.confidence = best_score;
          det.box = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
          det.center = cv::Point2f(cx, cy);
          candidates.push_back(det);
        }
      }
      continue;
    }

    // ---- yolov5（anchor-based，obj * cls）----
    const bool interleaved = (t.channels == v5_interleaved);
    if (!interleaved && t.channels != v5_split)
      continue;
    if (level < 0 || anchors_.size() < static_cast<size_t>((level * na + na) * 2))
      continue;

    for (int a = 0; a < na; ++a)
    {
      const float aw = anchors_[static_cast<size_t>((level * na + a) * 2 + 0)];
      const float ah = anchors_[static_cast<size_t>((level * na + a) * 2 + 1)];
      for (int y = 0; y < t.height; ++y)
      {
        for (int x = 0; x < t.width; ++x)
        {
          const size_t pixel = static_cast<size_t>(y) * t.width + x;
          size_t at;
          float objectness = 1.0f;
          if (interleaved)
          {
            at = (static_cast<size_t>(a) * (5 + nc)) * plane + pixel;
            objectness = 1.0f / (1.0f + std::exp(-t.data[at + 4 * plane]));
          }
          else
          {
            at = static_cast<size_t>(a) * 4 * plane + pixel;
          }
          if (objectness < confidence_threshold_)
            continue;

          float best_score = -1.0f;
          int best_class = -1;
          for (int c = 0; c < nc; ++c)
          {
            const size_t idx = interleaved
                                   ? at + static_cast<size_t>(5 + c) * plane
                                   : (static_cast<size_t>(na) * 4 + static_cast<size_t>(a) * nc + c) * plane + pixel;
            const float logit = t.data[idx];
            const float score = 1.0f / (1.0f + std::exp(-logit));
            if (score > best_score)
            {
              best_score = score;
              best_class = c;
            }
          }
          if (best_class < 0)
            continue;
          const float confidence = objectness * best_score;
          if (confidence < confidence_threshold_)
            continue;

          const float tx = 1.0f / (1.0f + std::exp(-t.data[at]));
          const float ty = 1.0f / (1.0f + std::exp(-t.data[at + plane]));
          const float tw = 1.0f / (1.0f + std::exp(-t.data[at + 2 * plane]));
          const float th = 1.0f / (1.0f + std::exp(-t.data[at + 3 * plane]));

          const float cx = (2.0f * tx - 0.5f + x) * stride;
          const float cy = (2.0f * ty - 0.5f + y) * stride;
          const float bw = (2.0f * tw) * (2.0f * tw) * aw;
          const float bh = (2.0f * th) * (2.0f * th) * ah;
          if (bw < 1.0f || bh < 1.0f)
            continue;

          Detection det;
          det.label = labels_[best_class];
          det.class_id = best_class;
          det.confidence = confidence;
          det.box = cv::Rect2f(cx - bw * 0.5f, cy - bh * 0.5f, bw, bh);
          det.center = cv::Point2f(cx, cy);
          candidates.push_back(det);
        }
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const Detection &a, const Detection &b) { return a.confidence > b.confidence; });
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    bool keep = true;
    for (size_t j = 0; j < results.size(); ++j)
    {
      if (results[j].class_id == candidates[i].class_id &&
          iou(results[j].box, candidates[i].box) > nms_iou_)
      {
        keep = false;
        break;
      }
    }
    if (keep)
      results.push_back(candidates[i]);
    if (static_cast<int>(results.size()) >= top_k_)
      break;
  }

  // letterbox 反变换：输入像素 -> 源图像素
  for (size_t i = 0; i < results.size(); ++i)
  {
    Detection &det = results[i];
    det.box.x = static_cast<float>((det.box.x - pad_x) / scale);
    det.box.y = static_cast<float>((det.box.y - pad_y) / scale);
    det.box.width = static_cast<float>(det.box.width / scale);
    det.box.height = static_cast<float>(det.box.height / scale);
    det.center = cv::Point2f(det.box.x + det.box.width * 0.5f,
                             det.box.y + det.box.height * 0.5f);
  }
  return results;
}

bool Detect::runYoloInference(const cv::Mat &canvas, std::vector<Tensor> &tensors)
{
#ifdef USE_RKNN
  cv::Mat rgb;
  cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);

  rknn_input in;
  memset(&in, 0, sizeof(in));
  in.index = 0;
  in.type = RKNN_TENSOR_UINT8;
  in.fmt = RKNN_TENSOR_NHWC;
  in.size = static_cast<uint32_t>(rgb.total() * rgb.elemSize());
  in.buf = rgb.data;
  in.pass_through = 0;
  if (rknn_inputs_set(ctx_, 1, &in) < 0)
  {
    ROS_ERROR_THROTTLE(2.0, "Detect: rknn_inputs_set failed (yolo)");
    return false;
  }
  if (rknn_run(ctx_, nullptr) < 0)
  {
    ROS_ERROR_THROTTLE(2.0, "Detect: rknn_run failed (yolo)");
    return false;
  }

  std::vector<rknn_output> outs(static_cast<size_t>(n_outputs_));
  memset(outs.data(), 0, sizeof(rknn_output) * outs.size());
  for (int i = 0; i < n_outputs_; ++i)
  {
    outs[i].index = static_cast<uint32_t>(i);
    outs[i].want_float = 1;
  }
  if (rknn_outputs_get(ctx_, n_outputs_, outs.data(), nullptr) < 0)
  {
    ROS_ERROR_THROTTLE(2.0, "Detect: rknn_outputs_get failed (yolo)");
    return false;
  }

  bool ok = true;
  for (int i = 0; i < n_outputs_ && ok; ++i)
  {
    const uint32_t *d = output_attr_[i].dims;
    if (d[1] < 1 || d[2] < 1 || d[3] < 1)
    {
      ok = false;
      break;
    }
    Tensor t;
    t.channels = static_cast<int>(d[1]);
    t.height = static_cast<int>(d[2]);
    t.width = static_cast<int>(d[3]);
    const size_t elems = static_cast<size_t>(t.channels) * t.height * t.width;
    const float *src = static_cast<const float *>(outs[i].buf);
    if (src == nullptr || elems == 0)
    {
      ok = false;
      break;
    }
    t.data.assign(src, src + elems);
    tensors.push_back(t);
  }
  rknn_outputs_release(ctx_, n_outputs_, outs.data());
  return ok;
#elif defined(USE_ONNX)
  cv::Mat blob;
  cv::dnn::blobFromImage(canvas, blob, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false,
                         CV_32F);
  net_.setInput(blob);
  std::vector<cv::Mat> outs;
  try
  {
    net_.forward(outs, out_names_);
  }
  catch (const cv::Exception &e)
  {
    ROS_ERROR_THROTTLE(2.0, "Detect: ONNX forward failed (yolo): %s", e.what());
    return false;
  }
  for (size_t i = 0; i < outs.size(); ++i)
  {
    const cv::Mat &m = outs[i];
    if (m.dims != 4 || m.type() != CV_32F || m.size[1] < 1 || m.size[2] < 1 || m.size[3] < 1)
      continue;
    Tensor t;
    t.channels = m.size[1];
    t.height = m.size[2];
    t.width = m.size[3];
    const size_t elems = static_cast<size_t>(t.channels) * t.height * t.width;
    t.data.assign(m.ptr<float>(), m.ptr<float>() + elems);
    tensors.push_back(t);
  }
  return !tensors.empty();
#else
  (void)canvas;
  (void)tensors;
  return false;
#endif
}

bool Detect::pixelToBody(const cv::Point2f &pixel, double height_m, cv::Point2f &body) const
{
  const cv::Size size = frameSize();
  if (size.width <= 0 || size.height <= 0 || height_m <= 0.0)
    return false;

  const double fx = camera_fx_ > 0.0
                        ? camera_fx_
                        : size.width / (2.0 * std::tan(camera_hfov_deg_ * M_PI / 360.0));
  const double fy = camera_fy_ > 0.0 ? camera_fy_ : fx;

  // 图像右 -> 机体右(-y)，图像下 -> 机体后(-x)
  const double right = (pixel.x - size.width * 0.5) * height_m / fx;
  const double down = (pixel.y - size.height * 0.5) * height_m / fy;
  double bx = -down;
  double by = -right;

  // 相机安装偏航：图像“上”方向相对机头的逆时针转角。
  const double a = camera_yaw_deg_ * M_PI / 180.0;
  body.x = static_cast<float>(std::cos(a) * bx - std::sin(a) * by);
  body.y = static_cast<float>(std::sin(a) * bx + std::cos(a) * by);
  return true;
}

void Detect::annotate(const Detection &det, const cv::Scalar &color, const std::string &tag,
                      double hold_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const ros::Time now = ros::Time::now();
  for (auto &annotation : annotations_)
  {
    if (annotation.tag == tag)
    {
      annotation.det = det;
      annotation.color = color;
      annotation.expires = now + ros::Duration(hold_sec);
      return;
    }
  }
  Annotation annotation;
  annotation.det = det;
  annotation.color = color;
  annotation.tag = tag;
  annotation.expires = now + ros::Duration(hold_sec);
  annotations_.push_back(annotation);
}

void Detect::clearAnnotations()
{
  std::lock_guard<std::mutex> lock(mutex_);
  annotations_.clear();
}

void Detect::publishCallback(const ros::TimerEvent &event)
{
  (void)event;
  cv::Mat frame;
  std::vector<Annotation> active;
  std::string frame_id = "camera";
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame_.empty())
      return;
    frame = frame_.clone();
    frame_id = frame_id_;
    const ros::Time now = ros::Time::now();
    for (auto it = annotations_.begin(); it != annotations_.end();)
    {
      if (now > it->expires)
        it = annotations_.erase(it);
      else
      {
        active.push_back(*it);
        ++it;
      }
    }
  }

  const cv::Size size = frame.size();
  const cv::Point2f center(size.width * 0.5f, size.height * 0.5f);
  cv::drawMarker(frame, center, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 18, 1);

  for (const auto &annotation : active)
  {
    const cv::Rect2f &box = annotation.det.box;
    cv::Rect rect(static_cast<int>(box.x), static_cast<int>(box.y),
                  std::max(1, static_cast<int>(box.width)),
                  std::max(1, static_cast<int>(box.height)));
    rect &= cv::Rect(0, 0, size.width, size.height);

    cv::rectangle(frame, rect, annotation.color, 2);
    std::ostringstream text;
    text << annotation.tag << " " << annotation.det.label << " "
         << static_cast<int>(annotation.det.confidence * 100) << "%";
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(text.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    const cv::Point origin(rect.x, std::max(text_size.height + 4, rect.y - 4));
    cv::putText(frame, text.str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, annotation.color, 1,
                cv::LINE_AA);
  }

  cv::Mat published = frame;
  if (publish_scale_ > 0.0 && std::fabs(publish_scale_ - 1.0) > 0.01)
    cv::resize(frame, published, cv::Size(), publish_scale_, publish_scale_);
  else
    published = frame;

  cv_bridge::CvImage out;
  out.header.stamp = ros::Time::now();
  out.header.frame_id = frame_id;
  out.encoding = sensor_msgs::image_encodings::BGR8;
  out.image = published;
  image_pub_.publish(out.toImageMsg());
}
