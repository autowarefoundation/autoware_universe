cmake_minimum_required(VERSION 3.14)
project(autoware_jerk_constant_deceleration_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_jerk_constant_deceleration_controller SHARED
  src/jerk_constant_deceleration_controller.cpp
)
ament_target_dependencies(autoware_jerk_constant_deceleration_controller)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::jerk_constant_deceleration_controller::JerkConstantDecelerationController"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
