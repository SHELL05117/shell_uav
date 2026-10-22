cmake_minimum_required(VERSION 3.0.2)
project(ego_planner_2d)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)
# normalize Eigen include dir variable: config mode (Eigen3Config.cmake) sets
# Eigen3_INCLUDE_DIRS, module mode (FindEigen3.cmake, e.g. on melodic) sets
# EIGEN3_INCLUDE_DIR. catkin_package(DEPENDS Eigen3) expects Eigen3_INCLUDE_DIRS.
if(NOT DEFINED Eigen3_INCLUDE_DIRS)
  set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bspline_opt path_searching grid_map_2d plan_manage
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# Core libraries (pure C++ / Eigen, no ROS dependency)
# ---------------------------------------------------------------------------
add_library(bspline_opt STATIC
  src/bspline_opt/uniform_bspline.cpp
  src/bspline_opt/bspline_optimizer.cpp
  src/bspline_opt/gradient_descent_optimizer.cpp
)

add_library(path_searching STATIC
  src/path_searching/dyn_a_star.cpp
)

add_library(grid_map_2d STATIC
  src/plan_env/grid_map.cpp
)

add_library(plan_manage STATIC
  src/plan_manage/planner_interface.cpp
)
target_link_libraries(plan_manage
  bspline_opt
  path_searching
  grid_map_2d
)

# ---------------------------------------------------------------------------
# Planner node
# ---------------------------------------------------------------------------
add_executable(ego_planner_2d_node src/ego_planner_2d_node.cpp)
target_link_libraries(ego_planner_2d_node
  plan_manage
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS bspline_opt path_searching grid_map_2d plan_manage
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ego_planner_2d_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
