cmake_minimum_required(VERSION 3.14)
project(autoware_topic_state_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/topic_state_monitor.cpp
  src/topic_state_monitor_core.cpp
)

autoware_agnocast_wrapper_setup(${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::topic_state_monitor::TopicStateMonitorNode"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
