cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

generate_parameter_library(${PROJECT_NAME}_param
  param/trajectory_adapter.param.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_adapter_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_adapter::TrajectoryAdapterNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
