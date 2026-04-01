cmake_minimum_required(VERSION 3.14)
project(autoware_evaluation_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/autoware_engage.cpp
  src/velocity_limit.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "autoware::evaluation_adapter::AutowareEngage"
  "autoware::evaluation_adapter::VelocityLimit"
)

ament_auto_package(INSTALL_TO_SHARE launch)
