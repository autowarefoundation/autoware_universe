cmake_minimum_required(VERSION 3.8)
project(autoware_command_mode_decider_plugins)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_command_mode_decider plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/command_mode_decider.cpp"
)

if(BUILD_TESTING)
endif()

ament_auto_package()
