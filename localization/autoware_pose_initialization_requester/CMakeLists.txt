cmake_minimum_required(VERSION 3.14)
project(autoware_pose_initialization_requester)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_initialization_requester.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::pose_initialization_requester::PoseInitializationRequester"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE launch)
