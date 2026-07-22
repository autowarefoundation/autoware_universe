cmake_minimum_required(VERSION 3.14)
project(autoware_lane_departure_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lane_departure_checker_node/lane_departure_checker_node.cpp
  src/lane_departure_checker_node/parameters.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lane_departure_checker::LaneDepartureCheckerNode"
  EXECUTABLE lane_departure_checker_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
