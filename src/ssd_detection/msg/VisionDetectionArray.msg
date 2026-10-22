# 一帧图像的全部检测结果。
#
# 刻意用自定义消息，而不是直接用 ssd_ros 上游的 object_msgs/ObjectsInBoxes：
#   * 本工程的工作区里没有 object_msgs，声明依赖会让整个工作区编不过；
#   * 上游那个消息只给归一化 roi，没有"这个框属于哪一类任务区"（zone）的信息，
#     而任务侧恰好只需要后者。
# 两个后端都直接发本消息：仿真 PC 用 OpenCV DNN（Python），Jetson 用本包自带的
# C++/detectNet 节点（src/ssd_detection_node.cpp + jetson_detector.cpp）。
# ⇒ 不再需要 ssd_ros，也不需要任何桥接节点，任务侧仿真与实机一致。

Header header

# 推理时所用帧的像素宽高。任务侧（MissionVision）靠它把归一化框换算成像素，
# 所以**每个发布方都必须填**：C++ 节点填相机实采尺寸，Python 节点填入站图尺寸。
# 0 = 发布方没给（老消息兼容路径），下游退回自己的 camera_width_/height_ 参数。
uint32 image_width
uint32 image_height

# 置信度 >= 阈值、按置信度降序排列的框。
VisionDetection[] detections
