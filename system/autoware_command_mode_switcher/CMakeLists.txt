cmake_minimum_required(VERSION 3.8)
project(autoware_command_mode_switcher)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/common/context.cpp"
  "src/common/plugin.cpp"
  "src/command_mode_switcher.cpp"
)
ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  "src/switchers/stop.cpp"
  "src/switchers/autonomous.cpp"
  "src/switchers/local.cpp"
  "src/switchers/remote.cpp"
  "src/switchers/emergency_stop.cpp"
  "src/switchers/comfortable_stop.cpp"
  "src/switchers/pull_over.cpp"
)
target_include_directories(${PROJECT_NAME} PRIVATE "src")
target_include_directories(${PROJECT_NAME}_plugins PRIVATE "src")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::command_mode_switcher::CommandModeSwitcher"
  EXECUTABLE "command_mode_switcher_node"
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
