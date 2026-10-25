cmake_minimum_required(VERSION 3.0.2)
project(navigation_2d)

add_compile_options(-std=c++11 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  mavros_msgs
  cv_bridge
  actionlib
  move_base_msgs
)
find_package(OpenCV REQUIRED)
message(STATUS "navigation_2d: OpenCV ${OpenCV_VERSION}")

# ---------------------------------------------------------------------------
# RKNN（RK3399Pro / TB-3399ProD）
#
# 板上运行时是 librknn_runtime.so（rknn-toolkit 1.7.5 那一套），头文件 rknn_api.h。
# 不是 RK356x/3588 的 librknnrt，后者和 RK3399Pro 的 NPU 驱动不兼容。
# 纯 x86 上只跑航点/仿真、不接相机时用 -DUSE_RKNN=OFF 也能编。
# ---------------------------------------------------------------------------
option(USE_RKNN "Build the RKNN inference backend (RK3399Pro)" ON)
set(RKNN_LIBRARY "")
if(USE_RKNN)
  find_path(RKNN_INCLUDE_DIR rknn_api.h
    HINTS /usr/include /usr/local/include /usr/include/rknn /opt/rknn/include)
  find_library(RKNN_LIBRARY NAMES rknn_runtime rknnrt rknn_api
    HINTS /usr/lib /usr/lib/aarch64-linux-gnu /usr/local/lib /opt/rknn/lib)
  if(NOT RKNN_INCLUDE_DIR OR NOT RKNN_LIBRARY)
    message(FATAL_ERROR
      "RKNN (rknn_api.h / librknn_runtime.so) not found.\n"
      "  On the TB-3399ProD install rknn-toolkit 1.7.5's runtime, or build with\n"
      "  catkin build navigation_2d --cmake-args -DUSE_RKNN=OFF\n"
      "  (waypoints and drops still work, detection is disabled).")
  endif()
  message(STATUS "navigation_2d: RKNN ${RKNN_LIBRARY}, headers ${RKNN_INCLUDE_DIR}")
  add_definitions(-DUSE_RKNN)
  include_directories(${RKNN_INCLUDE_DIR})
endif()

# ---------------------------------------------------------------------------
# ONNX 后端（x86 / Gazebo 仿真）
#
# 用 OpenCV DNN 读同一个 raw-head YOLO onnx（detect_mode/26/gc-yolo-raw-*.onnx）：
# 不用 NPU、不用先转 .rknn，识别 -> 标注 -> 视觉对准 -> 投放整条链都能在仿真里过一遍。
# 板上 USE_RKNN=ON 时自动不编这个（一个可执行文件里只放一个后端，省得两份
# 推理代码互相干扰；运行时还能用 ~backend=rknn|onnx|none 覆盖）。
# CMake 3.0 没有 IN_LIST，用 list(FIND) 判 dnn 模块。
# ---------------------------------------------------------------------------
option(USE_ONNX "Run the ONNX model through OpenCV DNN (x86 simulation, no NPU)" ON)
if(NOT USE_RKNN AND USE_ONNX)
  list(FIND OpenCV_LIB_COMPONENTS dnn _opencv_dnn_index)
  if(_opencv_dnn_index GREATER -1 OR TARGET opencv_dnn)
    add_definitions(-DUSE_ONNX)
    message(STATUS "navigation_2d: ONNX backend via OpenCV DNN (simulation)")
  else()
    message(WARNING "navigation_2d: OpenCV has no dnn module; ONNX backend disabled")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs mavros_msgs cv_bridge
                 actionlib move_base_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(detectnet src/detect_main.cpp src/detect.cpp)
target_link_libraries(detectnet ${catkin_LIBRARIES} ${OpenCV_LIBS} ${RKNN_LIBRARY})

add_executable(shoot_with_move_base_node
  src/detect.cpp
  src/flight_control.cpp
  src/shoot_with_move_base.cpp
)
target_link_libraries(shoot_with_move_base_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${RKNN_LIBRARY})

install(TARGETS detectnet shoot_with_move_base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch param docs detect_mode
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
