cmake_minimum_required(VERSION 3.8)
project(xiaou_arm_planning)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(target_planner_node src/target_planner_node.cpp)
ament_target_dependencies(
  target_planner_node geometry_msgs moveit_msgs moveit_ros_planning_interface rclcpp std_msgs
)

install(TARGETS target_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
