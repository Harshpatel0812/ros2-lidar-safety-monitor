cmake_minimum_required(VERSION 3.8)
project(robot_safety_monitor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(safety_monitor_node src/safety_monitor.cpp)
target_compile_features(safety_monitor_node PUBLIC cxx_std_17)
target_include_directories(safety_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(safety_monitor_node rclcpp sensor_msgs std_msgs std_srvs)

install(TARGETS safety_monitor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/velocity_guard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME velocity_guard)
install(PROGRAMS
  scripts/demo_scan_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_scan_publisher)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_safety_logic test/test_safety_logic.cpp)
  target_include_directories(test_safety_logic PRIVATE include)
  target_compile_features(test_safety_logic PUBLIC cxx_std_17)
endif()

ament_package()
