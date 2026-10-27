cmake_minimum_required(VERSION 2.8.12)
project(usb_cam)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp std_msgs std_srvs sensor_msgs camera_info_manager)

# Find OpenCV
find_package(OpenCV REQUIRED)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(avutil libavutil REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
  ${avutil_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp src/rockchip_mjpeg.cpp)

# Optional, isolated RK runtime; other platforms retain the FFmpeg backend.
find_path(MPP_INCLUDE_DIR rk_mpi.h PATHS /opt/shell-media/include/rockchip)
find_path(RGA_INCLUDE_DIR RgaApi.h PATHS /opt/shell-media/include /opt/shell-media/include/rga)
find_library(MPP_LIBRARY rockchip_mpp PATHS /opt/shell-media/lib)
find_library(RGA_LIBRARY rga PATHS /opt/shell-media/lib)
if(MPP_INCLUDE_DIR AND RGA_INCLUDE_DIR AND MPP_LIBRARY AND RGA_LIBRARY)
  target_compile_definitions(${PROJECT_NAME} PRIVATE USB_CAM_ROCKCHIP_MEDIA)
  target_include_directories(${PROJECT_NAME} PRIVATE ${MPP_INCLUDE_DIR} ${RGA_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${MPP_LIBRARY} ${RGA_LIBRARY})
  message(STATUS "usb_cam: MPP/RGA backend enabled")
endif()
target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(usb_cam_lifecycle_test test/lifecycle_test.cpp)
  if(TARGET usb_cam_lifecycle_test)
    target_link_libraries(usb_cam_lifecycle_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
