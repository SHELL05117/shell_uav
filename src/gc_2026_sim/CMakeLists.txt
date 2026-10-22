cmake_minimum_required(VERSION 3.0.2)
project(gc_2026_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS gazebo_msgs geometry_msgs mavros_msgs nav_msgs roscpp std_msgs tf
                 visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(gazebo_simulation_vehicle_node src/simulation_vehicle.cpp)
target_link_libraries(gazebo_simulation_vehicle_node ${catkin_LIBRARIES})
add_dependencies(gazebo_simulation_vehicle_node ${catkin_EXPORTED_TARGETS})

add_executable(px4_sensor_rig_node src/px4_sensor_rig.cpp)
target_link_libraries(px4_sensor_rig_node ${catkin_LIBRARIES})
add_dependencies(px4_sensor_rig_node ${catkin_EXPORTED_TARGETS})

install(TARGETS gazebo_simulation_vehicle_node px4_sensor_rig_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch gazebo param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/check_altitude.py
  scripts/check_px4_field.py
  scripts/check_px4_slam.py
  scripts/install_px4_field_world.sh
  scripts/px4_field_rcS.sh
  scripts/start_px4_slam.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
