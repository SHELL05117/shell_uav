cmake_minimum_required(VERSION 3.0.2)
project(location)

add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# TFmini 的 I2C 访问用 libi2c（i2c-tools），不要再写死 aarch64 下的绝对路径：
# Jetson 和 3399Pro 的多架构库目录不一定同名。
find_library(I2C_LIBRARY NAMES i2c)
if(NOT I2C_LIBRARY)
  message(FATAL_ERROR "libi2c not found (install i2c-tools / libi2c-dev)")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(location_node
  src/location_with_ftmini.cpp
  src/TFplus.cpp
  src/i2c.cpp
  src/tf_mini_i2c.cpp
)
target_link_libraries(location_node ${catkin_LIBRARIES} ${I2C_LIBRARY})

add_executable(tfmini_test
  src/tfmini_test.cpp
  src/TFplus.cpp
  src/i2c.cpp
  src/tf_mini_i2c.cpp
)
target_link_libraries(tfmini_test ${catkin_LIBRARIES} ${I2C_LIBRARY})

if(CATKIN_ENABLE_TESTING)
  find_package(Threads REQUIRED)
  find_library(UTIL_LIBRARY NAMES util)
  catkin_add_gtest(localization_guard_test test/localization_guard_test.cpp)
  if(TARGET localization_guard_test)
    target_link_libraries(localization_guard_test ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(tfmini_uart_pty_test test/tfmini_uart_pty_test.cpp src/TFplus.cpp)
  if(TARGET tfmini_uart_pty_test)
    target_link_libraries(tfmini_uart_pty_test ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${UTIL_LIBRARY})
  endif()
endif()

install(TARGETS location_node tfmini_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/monitor_jpeg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
