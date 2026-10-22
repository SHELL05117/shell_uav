cmake_minimum_required(VERSION 3.0.2)
project(ssd_detection)

## 消息 + 两个后端：
##   * scripts/ssd_detection_node.py   —— onnxruntime / OpenCV DNN，仿真 PC 用（没有 CUDA）
##   * src/ssd_detection_node.cpp      —— jetson-inference detectNet，Jetson 用
## 两个都发 ssd_detection/VisionDetectionArray，所以下游不区分平台。
##
## 模型只有一个：**.onnx**。仿真与实机读同一个文件（Python 节点走 onnxruntime，
## Jetson 走 detectNet），所以"仿真里过的那套权重"与"飞的时候用的那套权重"
## 不可能是两份不同的东西 —— 这是相对"每个平台各存一份 .pb / .plan"最大的
## 好处。读它之前先过一遍 tools/ssd_training/check_onnx_contract.py。

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  cv_bridge
  std_msgs
)

add_message_files(
  FILES
  VisionDetection.msg
  VisionDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs cv_bridge std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 只有仿真侧那个 Python 节点。装到 lib/<pkg> 便于 launch 里用 type=。
catkin_install_python(PROGRAMS
  scripts/ssd_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## 模型与标签随包安装：launch 里的默认路径是 $(find ssd_detection)/models/...，
## 只装 launch 的话 install 空间下这两个文件会消失，而 devel 空间下它们一直在
## （rospack find 指向源码目录）—— 那种"源码里跑得动、装出来跑不动"的差别
## 正是门禁类错误里最难查的一种，所以这里显式装上。
install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.onnx" PATTERN "*.txt"
)

############################################################
## jetson-inference 版（只在装了它的 Jetson 上构建）        ##
############################################################
## 仿真 PC 上没有 jetson-inference，所以这一段必须是**可选**的，否则整个工作区
## 在 PC 上就编不过。找不到就跳过，并打印一句说明，免得以为是漏配了什么。
##
## 两个 find_package 的**顺序不能换**：jetson-inference 的 config 会去查
## jetson-utils，没先 find 过它就会报错（jetson-inference 官方 CMake 的要求）。
## 头文件装在 /usr/local/include，所以要显式把它加进 include 路径。
##
## 这里用 add_executable 而不是 cuda_add_executable：jetson_detector.cpp 自己
## 不写任何 CUDA 代码，nvInfer.h 之类只出现在 jetson-inference 的头里，
## 用 g++ 编就够了。改动前那版需要 nvcc，是因为它自己 include 了
## cuda_runtime.h 来做 NMS 的后处理 —— detectNet 把那部分接过去了。

find_package(OpenCV QUIET)
find_package(jetson-utils QUIET)
find_package(jetson-inference QUIET)
find_package(CUDA QUIET)

set(SSD_DETECTION_JETSON_AVAILABLE FALSE)
if(jetson-inference_FOUND AND OpenCV_FOUND AND CUDA_FOUND)
  set(SSD_DETECTION_JETSON_AVAILABLE TRUE)
endif()

if(SSD_DETECTION_JETSON_AVAILABLE)
  message(STATUS "ssd_detection: jetson-inference found -> building the detectNet node")
  # /usr/local/include/jetson-utils：新版把 utils 拆成了独立包，videoSource.h 之类
  # 装在这里（旧版是平铺进 include/jetson-inference/）。两个都加上，源码里的
  # __has_include 才能两种布局都吃得下；不存在的目录 gcc 直接忽略，不会报错。
  include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS}
                      /usr/local/include /usr/local/include/jetson-utils
                      /usr/local/include/jetson-inference)

  # libnvbuf_utils.so 在 /usr/lib/aarch64-linux-gnu/tegra/（JetPack 4.x），不在 ld
  # 默认搜索路径里；链接 jetson-inference 时会传递性带进 -lnvbuf_utils（来自
  # jetson-utils 的多媒体代码），缺这行就报 "cannot find -lnvbuf_utils"。
  # 照抄旧工程 catkin_ws_bak/navigation_2d/CMakeLists.txt 的做法（实机验证过）。
  if(EXISTS /usr/lib/aarch64-linux-gnu/tegra)
    link_directories(/usr/lib/aarch64-linux-gnu/tegra)
  endif()

  add_executable(ssd_detection_node
    src/ssd_detection_node.cpp
    src/jetson_detector.cpp
  )
  add_dependencies(ssd_detection_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(ssd_detection_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    jetson-inference
  )

  install(TARGETS ssd_detection_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(STATUS "ssd_detection: jetson-inference not found -> skipping the detectNet "
                 "node (this is expected off the Jetson; the Python node covers the "
                 "simulator, and both read the same .onnx)")
endif()
