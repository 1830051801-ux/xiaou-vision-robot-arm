cmake_minimum_required(VERSION 3.16)
project(xiaou_arm_can_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(xiaou_arm_can_system SHARED
  src/xiaou_arm_can_system.cpp
)
target_compile_features(xiaou_arm_can_system PUBLIC cxx_std_17)
target_include_directories(xiaou_arm_can_system PUBLIC include)
ament_target_dependencies(
  xiaou_arm_can_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface xiaou_arm_can_control.xml)

install(
  TARGETS xiaou_arm_can_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES xiaou_arm_can_control.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
