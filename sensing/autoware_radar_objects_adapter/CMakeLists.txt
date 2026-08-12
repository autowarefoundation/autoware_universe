cmake_minimum_required(VERSION 3.14)
project(autoware_radar_objects_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/radar_objects_adapter.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::RadarObjectsAdapter"
  EXECUTABLE radar_objects_adapter_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
)

ament_auto_package(INSTALL_TO_SHARE launch config)
