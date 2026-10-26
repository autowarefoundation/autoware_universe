cmake_minimum_required(VERSION 3.14)
project(autoware_path_distance_calculator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ros_interface.cpp
  src/main.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::path_distance_calculator::PathDistanceCalculator"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

ament_auto_package(INSTALL_TO_SHARE launch)
