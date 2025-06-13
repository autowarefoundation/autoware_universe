cmake_minimum_required(VERSION 3.14)
project(autoware_radar_objects_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/radar_objects_adapter.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::RadarObjectsAdapter"
  EXECUTABLE radar_objects_adapter_node
)

ament_auto_package(INSTALL_TO_SHARE launch config)
