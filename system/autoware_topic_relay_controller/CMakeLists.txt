cmake_minimum_required(VERSION 3.14)
project(autoware_topic_relay_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/topic_relay_controller_node.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::topic_relay_controller::TopicRelayController"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
  ROS2_EXECUTOR MultiThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
