cmake_minimum_required(VERSION 3.8)
project(drone_dynamics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(
  quadrotor_dynamics_node
  src/quadrotor_dynamics_node.cpp
)

ament_target_dependencies(
  quadrotor_dynamics_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
)

target_link_libraries(
  quadrotor_dynamics_node
  Eigen3::Eigen
)

install(
  TARGETS quadrotor_dynamics_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
