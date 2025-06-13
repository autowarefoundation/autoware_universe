cmake_minimum_required(VERSION 3.14)
project(autoware_topic_state_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/topic_state_monitor/topic_state_monitor.cpp
  src/topic_state_monitor_core.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::topic_state_monitor::TopicStateMonitorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
