cmake_minimum_required(VERSION 3.14)
project(autoware_duplicated_node_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(rclcpp REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/duplicated_node_checker_core.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::duplicated_node_checker::DuplicatedNodeChecker"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
