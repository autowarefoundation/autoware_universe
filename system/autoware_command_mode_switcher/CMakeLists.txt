cmake_minimum_required(VERSION 3.8)
project(autoware_command_mode_switcher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/common/command_container.cpp"
  "src/common/selector_interface.cpp"
  "src/plugin/command_plugin.cpp"
  "src/command_mode_switcher.cpp"
)
target_include_directories(${PROJECT_NAME} PRIVATE "src")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::command_mode_switcher::CommandModeSwitcher"
  EXECUTABLE "command_mode_switcher_node"
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
