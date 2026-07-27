cmake_minimum_required(VERSION 3.14)
project(autoware_hazard_lights_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::hazard_lights_selector::HazardLightsSelector"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
