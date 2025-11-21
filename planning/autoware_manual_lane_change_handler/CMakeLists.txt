cmake_minimum_required(VERSION 3.14)
project(autoware_manual_lane_change_handler)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/manual_lane_change_handler.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  autoware_planning_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::manual_lane_change_handler::ManualLaneChangeHandler"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
