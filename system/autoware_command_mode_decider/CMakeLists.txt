cmake_minimum_required(VERSION 3.8)
project(autoware_command_mode_decider)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/plugin/plugin.cpp"
  "src/plugin/status.cpp"
  "src/command_mode_decider_base.cpp"
)
target_include_directories(${PROJECT_NAME} PRIVATE "src")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::command_mode_decider::CommandModeDeciderBase"
  EXECUTABLE "command_mode_decider_node"
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
