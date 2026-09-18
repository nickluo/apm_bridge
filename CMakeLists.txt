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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavlink REQUIRED)

include_directories(include)

add_library(xfgimbal STATIC
  src/gimbal.cpp
  src/serial/serial.cc
  src/serial/unix.cc
  src/serial/list_ports_linux.cc
)
ament_target_dependencies(xfgimbal
  rclcpp
  tf2_ros
  sensor_msgs
)

add_executable(apm_bridge_node src/apm_vehicle_node.cpp)
ament_target_dependencies(apm_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  quadrotor_msgs
  mavros_msgs
  mavros
  mavlink
)

target_link_libraries(apm_bridge_node xfgimbal)

# Betaflight custom-link bridge: same ROS interface as apm_bridge_node, but
# talks the firmware's dedicated binary link (USE_CUSTOM_LINK) over a serial
# port or TCP (SITL) instead of MAVLink through MAVROS.
add_executable(custom_link_bridge_node
  src/custom_link_vehicle_node.cpp
  src/custom_link/client.cpp
  src/custom_link/transport.cpp
)
ament_target_dependencies(custom_link_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  quadrotor_msgs
  mavros_msgs
)

target_link_libraries(custom_link_bridge_node xfgimbal)

install(TARGETS
  apm_bridge_node
  custom_link_bridge_node
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

# 台架标定工具 (kestrel_utils 移植): thrust_test_node + scale_reader_node
install(PROGRAMS scripts/thrust_test_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME thrust_test_node)
install(PROGRAMS scripts/scale_reader_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME scale_reader_node)

ament_package()