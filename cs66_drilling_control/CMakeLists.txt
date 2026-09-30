cmake_minimum_required(VERSION 3.8)
project(cs66_drilling_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(drilling_approach
  src/drilling_approach.cpp
)

ament_target_dependencies(
  drilling_approach
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(drilling_retract
  src/drilling_retract.cpp
)

ament_target_dependencies(
  drilling_retract
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(drilling_contact_test
  src/drilling_contact_test.cpp
)

ament_target_dependencies(
  drilling_contact_test
  rclcpp
  geometry_msgs
)

add_executable(drilling_force_approach
  src/drilling_force_approach.cpp
)

ament_target_dependencies(
  drilling_force_approach
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)
add_executable(drilling_cycle
  src/drilling_cycle.cpp
)

ament_target_dependencies(
  drilling_cycle
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)
install(
  TARGETS
    drilling_approach
    drilling_retract
    drilling_contact_test
    drilling_force_approach
    drilling_cycle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
