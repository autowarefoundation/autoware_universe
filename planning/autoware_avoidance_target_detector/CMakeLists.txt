cmake_minimum_required(VERSION 3.14)
project(autoware_avoidance_target_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/object_filtering.cpp
  src/boundary.cpp
  src/parameter.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::avoidance_target_detector::AvoidanceTargetDetectorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
