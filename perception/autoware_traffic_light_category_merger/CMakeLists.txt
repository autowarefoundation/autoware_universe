cmake_minimum_required(VERSION 3.8)
project(autoware_traffic_light_category_merger)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_category_merger_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::TrafficLightCategoryMergerNode"
  EXECUTABLE traffic_light_category_merger_node)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
