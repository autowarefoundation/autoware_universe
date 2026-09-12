cmake_minimum_required(VERSION 3.14)
project(autoware_diagnostic_graph_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lib/graph.cpp
  src/lib/subscription.cpp
)
autoware_agnocast_wrapper_setup(${PROJECT_NAME})

ament_auto_add_library(${PROJECT_NAME}_tools SHARED
  src/node/dump.cpp
  src/node/converter.cpp
  src/node/logging.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}_tools
  PLUGIN "autoware::diagnostic_graph_utils::DumpNode"
  EXECUTABLE dump_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}_tools
  PLUGIN "autoware::diagnostic_graph_utils::ConverterNode"
  EXECUTABLE converter_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}_tools
  PLUGIN "autoware::diagnostic_graph_utils::LoggingNode"
  EXECUTABLE logging_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE launch)
