cmake_minimum_required(VERSION 2.8.3)
project(location)
 
## The location node uses std::unique_ptr/std::make_unique/std::exchange.
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(I2C_LIBRARY
  NAMES i2c libi2c.so.0 i2c.so.0
  PATHS
    /usr/lib/aarch64-linux-gnu
    /lib/aarch64-linux-gnu
    /usr/lib
    /lib
)

if(NOT I2C_LIBRARY)
  message(FATAL_ERROR "Could not find libi2c. Install the libi2c development/runtime package before building location.")
endif()

## Generate services in the 'srv' folder
# add_service_files(FILES AddTwoInts.srv)
# generate_messages(DEPENDENCIES std_msgs)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

#catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
 
add_executable(location_node src/location_with_ftmini.cpp src/TFplus.cpp src/i2c.cpp src/tf_mini_i2c.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(location_node ${catkin_LIBRARIES} ${I2C_LIBRARY})


add_executable(tfmini_test src/tfmini_test.cpp src/TFplus.cpp src/i2c.cpp src/tf_mini_i2c.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tfmini_test ${catkin_LIBRARIES} ${I2C_LIBRARY})


## Declare a cpp executable
# add_executable(position_with_tfmini_node src/position_with_tfmini.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(position_with_tfmini_node
#   ${catkin_LIBRARIES}
# )


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )
## Mark executables and/or libraries for installation
# install(TARGETS  position_to_mavros_node
#   set_mode_node
#   collision_avoidance_vfh_node
#   avoidance_car_vfh_node
#   collision_avoidance_matlabvfh_node
#   circular_node circular_car_node 
#   ros_nav_quadrotor_node 
#   mission_car_offboard_node
#   landing_quadrotor_node
#   tracking_quadrotor_node
#   tracking_down_kcf_quadrotor_node
#   tracking_ardupilot_node
#   auto_2d_nav_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY
#   launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )




