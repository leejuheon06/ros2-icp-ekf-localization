cmake_minimum_required(VERSION 3.8)

project(localization_evaluation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(
  ground_truth_node
  src/ground_truth_node.cpp
)

ament_target_dependencies(
  ground_truth_node
  rclcpp
  tf2_msgs
  geometry_msgs
)

add_executable(
  localization_evaluation_node
  src/localization_evaluation_node.cpp
)

ament_target_dependencies(
  localization_evaluation_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
)

add_executable(
  localization_benchmark_runner
  src/localization_benchmark_runner.cpp
)

ament_target_dependencies(
  localization_benchmark_runner
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  nav2_msgs
  lifecycle_msgs
)

install(
  TARGETS
    ground_truth_node
    localization_evaluation_node
    localization_benchmark_runner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
