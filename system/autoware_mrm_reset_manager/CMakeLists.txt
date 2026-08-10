cmake_minimum_required(VERSION 3.14)
project(autoware_mrm_reset_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mrm_reset_manager/mrm_reset_manager.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::mrm_reset_manager::MrmResetManager"
  EXECUTOR "MultiThreadedExecutor"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE launch config)
