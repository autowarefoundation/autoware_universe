cmake_minimum_required(VERSION 3.14)
project(autoware_planning_validator_intersection_collision_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_planning_validator plugins.xml)

include_directories(
  ${autoware_planning_validator_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  ${autoware_planning_validator_LIBRARIES}
)

ament_auto_package(INSTALL_TO_SHARE config)
