cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(navigation_2d)
 
## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  actionlib_msgs
  tf
  ssd_detection
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

add_message_files(
  FILES
  TargetDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

#catkin_python_setup()

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES vision_to_mavros
  CATKIN_DEPENDS roscpp message_runtime geometry_msgs mavros_msgs nav_msgs sensor_msgs std_msgs actionlib_msgs tf ssd_detection
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include
)

add_executable(rescue_mission_node src/mission_vision.cpp src/flight_control.cpp src/rescue_mission.cpp )

target_link_libraries(rescue_mission_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(rescue_mission_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ego_trajectory_controller_node src/ego_trajectory_controller.cpp)
target_link_libraries(ego_trajectory_controller_node ${catkin_LIBRARIES})

add_dependencies(ego_trajectory_controller_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(ego_path_tracking_test test/ego_path_tracking_test.cpp)
  if(TARGET ego_path_tracking_test)
    target_link_libraries(ego_path_tracking_test ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS rescue_mission_node ego_trajectory_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# param/ 与 scripts/ 已随 DWA/TEB/move_base/HSV 一起删除（2026-10-17），
# 这个包现在只剩 launch 需要安装。
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
