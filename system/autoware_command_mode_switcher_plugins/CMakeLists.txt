cmake_minimum_required(VERSION 3.8)
project(autoware_command_mode_switcher_plugins)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_command_mode_switcher plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/stop.cpp"
  "src/autonomous.cpp"
  "src/local.cpp"
  "src/remote.cpp"
  "src/emergency_stop.cpp"
  "src/comfortable_stop.cpp"
  "src/pull_over.cpp"
  "src/main_ecu_in_lane_emergency_stop.cpp"
  "src/main_ecu_in_lane_moderate_stop.cpp"
  "src/sub_ecu_in_lane_moderate_stop.cpp"
  "src/sub_ecu_standby.cpp"
)

if(BUILD_TESTING)
endif()

ament_auto_package()
