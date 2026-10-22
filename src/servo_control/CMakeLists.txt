cmake_minimum_required(VERSION 3.0.2)
project(servo_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(servo_controller_node src/servo_controller_node.cpp)
target_link_libraries(servo_controller_node ${catkin_LIBRARIES})
add_dependencies(servo_controller_node ${catkin_EXPORTED_TARGETS})

install(TARGETS servo_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# servo_bench.launch 是台架标定入口（angle_debug=true 的唯一去处）。
# 必须 install，否则装过的工作空间里 roslaunch 找不到它。
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
