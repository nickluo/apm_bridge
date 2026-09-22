cmake_minimum_required(VERSION 3.10)
project(apm_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

include_directories(include)

# XFrobot 云台驱动 (私有协议, 支持 C-20S/C-40D/C-200T)
add_library(xfgimbal STATIC
  src/gimbal.cpp
  src/serial/serial.cc
  src/serial/unix.cc
  src/serial/list_ports_linux.cc
)
target_link_libraries(xfgimbal
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  sensor_msgs::sensor_msgs
)

# Betaflight custom-link bridge: 与定制化 Betaflight 固件的专用二进制链路
# (USE_CUSTOM_LINK, serial/TCP-SITL) 桥接; 云台经私有协议直驱 (默认 C-20S)。
add_executable(custom_link_bridge_node
  src/custom_link_vehicle_node.cpp
  src/custom_link/client.cpp
  src/custom_link/transport.cpp
)
target_link_libraries(custom_link_bridge_node
  rclcpp::rclcpp
  std_msgs::std_msgs
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  sensor_msgs::sensor_msgs
  quadrotor_msgs::quadrotor_msgs
  mavros_msgs::mavros_msgs
)

target_link_libraries(custom_link_bridge_node xfgimbal)

# 云台协议自检 harness: 供 test/gimbal_pysim_test.py 在 pty 上做协议级仿真验证
add_executable(gimbal_selftest test/gimbal_selftest.cpp)
target_link_libraries(gimbal_selftest
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  sensor_msgs::sensor_msgs
)
target_link_libraries(gimbal_selftest xfgimbal)

install(TARGETS
  custom_link_bridge_node
  gimbal_selftest
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  parameters
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
