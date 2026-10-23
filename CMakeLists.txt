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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

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
  tf2_geometry_msgs::tf2_geometry_msgs
)

# Betaflight custom-link bridge: 与定制化 Betaflight 固件的专用二进制链路
# (USE_CUSTOM_LINK, serial/TCP-SITL) 桥接; 云台经私有协议直驱 (默认 C-20S)。
add_executable(custom_link_bridge_node
  src/custom_link_vehicle_node.cpp
  src/airsim_gimbal.cpp
  src/gimbal_factory.cpp
  src/strapdown_gimbal.cpp
  src/custom_link/sitl_transport.cpp
  src/custom_link/client.cpp
  src/custom_link/transport.cpp
)
target_link_libraries(custom_link_bridge_node
  rclcpp::rclcpp
  std_msgs::std_msgs
  geometry_msgs::geometry_msgs
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  sensor_msgs::sensor_msgs
  quadrotor_msgs::quadrotor_msgs
  mavros_msgs::mavros_msgs
  nav_msgs::nav_msgs
)

target_link_libraries(custom_link_bridge_node xfgimbal)

set(AIRSIM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../cosys_airsim" CACHE PATH
    "Cosys-AirSim checkout containing rpclib sources for the gimbal API")
option(APM_BRIDGE_AIRSIM "Build AirSim gimbal API support in CustomLinkVehicleNode" OFF)
if(APM_BRIDGE_AIRSIM)
  if(NOT EXISTS "${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.1/CMakeLists.txt")
    message(FATAL_ERROR "AIRSIM_ROOT must contain external/rpclib/rpclib-2.3.1")
  endif()
  # Compile with the ROS toolchain: Unreal's prebuilt libc++ SDK has a different ABI.
  add_subdirectory("${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.1" airsim_rpc EXCLUDE_FROM_ALL)
  target_compile_definitions(custom_link_bridge_node PRIVATE APM_BRIDGE_AIRSIM)
  target_link_libraries(custom_link_bridge_node rpc)
endif()

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

install(PROGRAMS scripts/sitl_motor_calibration.py
  DESTINATION lib/${PROJECT_NAME} RENAME sitl_motor_calibration)
install(FILES scripts/sitl_support.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/bench_pid_tuner.py
  DESTINATION lib/${PROJECT_NAME} RENAME bench_pid_tuner)
install(FILES scripts/bench_pid_core.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(gimbal_tracking_test test/gimbal_tracking_test.cpp)
  add_test(NAME gimbal_tracking_test COMMAND gimbal_tracking_test)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sitl_bridge_test test/sitl_bridge_test.py)
  ament_add_pytest_test(bench_pid_test test/bench_pid_test.py TIMEOUT 240)
  ament_add_pytest_test(bench_angle_link_test test/bench_angle_link_test.py TIMEOUT 30)
  ament_add_pytest_test(gimbal_node_test test/gimbal_node_test.py TIMEOUT 30)
  set_tests_properties(gimbal_node_test PROPERTIES
    ENVIRONMENT "APM_BRIDGE_TEST_BINARY=$<TARGET_FILE:custom_link_bridge_node>")
  ament_add_pytest_test(gimbal_protocol_test test/gimbal_pysim_test.py TIMEOUT 45)
  set_tests_properties(gimbal_protocol_test PROPERTIES
    ENVIRONMENT "GIMBAL_SELFTEST=$<TARGET_FILE:gimbal_selftest>")
  add_executable(sitl_relay_test
    test/sitl_relay_test.cpp src/custom_link/sitl_transport.cpp
    src/custom_link/transport.cpp src/custom_link/client.cpp)
  target_link_libraries(sitl_relay_test xfgimbal rclcpp::rclcpp)
  add_test(NAME sitl_relay_test COMMAND sitl_relay_test)
  set_tests_properties(sitl_relay_test PROPERTIES TIMEOUT 10)
  add_executable(gimbal_interface_test
    test/gimbal_interface_test.cpp src/gimbal_factory.cpp src/airsim_gimbal.cpp src/strapdown_gimbal.cpp)
  target_link_libraries(gimbal_interface_test xfgimbal rclcpp::rclcpp tf2_geometry_msgs::tf2_geometry_msgs)
  if(APM_BRIDGE_AIRSIM)
    target_compile_definitions(gimbal_interface_test PRIVATE APM_BRIDGE_AIRSIM)
    target_link_libraries(gimbal_interface_test rpc)
  endif()
  add_test(NAME gimbal_interface_test COMMAND gimbal_interface_test)
  set_tests_properties(gimbal_interface_test PROPERTIES TIMEOUT 60)
  option(APM_BRIDGE_AIRSIM_LIVE_TEST "Enable read-only reconstruction test against a running AirSim instance" OFF)
  if(APM_BRIDGE_AIRSIM AND APM_BRIDGE_AIRSIM_LIVE_TEST)
    add_test(NAME gimbal_airsim_live_test COMMAND gimbal_interface_test --airsim-live)
    set_tests_properties(gimbal_airsim_live_test PROPERTIES TIMEOUT 30)
  endif()
endif()

ament_package()
