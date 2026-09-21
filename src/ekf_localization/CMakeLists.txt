cmake_minimum_required(VERSION 3.8)
project(ekf_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(
  ekf_localization_node
  src/ekf_localization_node.cpp
)

ament_target_dependencies(
  ekf_localization_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
)

target_link_libraries(
  ekf_localization_node
  Eigen3::Eigen
)

install(
  TARGETS
  ekf_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
