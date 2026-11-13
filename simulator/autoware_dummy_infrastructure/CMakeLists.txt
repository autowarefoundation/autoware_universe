cmake_minimum_required(VERSION 3.14)
project(autoware_dummy_infrastructure)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_dummy_infrastructure_node_component SHARED
  src/dummy_infrastructure_node/dummy_infrastructure_node.cpp
)

rclcpp_components_register_node(autoware_dummy_infrastructure_node_component
  PLUGIN "autoware::dummy_infrastructure::DummyInfrastructureNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
