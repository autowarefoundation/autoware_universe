cmake_minimum_required(VERSION 3.14)
project(autoware_redundancy_command_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/redundancy_command_selector.hpp
  src/redundancy_command_selector.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::simulator::redundancy_command_selector::RedundancyCommandSelector"
  EXECUTABLE redundancy_command_selector_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR MultiThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE launch)
