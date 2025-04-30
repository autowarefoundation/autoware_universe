cmake_minimum_required(VERSION 3.14)
project(autoware_planning_latency_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()
# find_package(autoware_planning_validator REQUIRED)
pluginlib_export_plugin_description_file(autoware_planning_validator plugins.xml)

include_directories(
  ${autoware_planning_validator_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

ament_auto_package(INSTALL_TO_SHARE config)
