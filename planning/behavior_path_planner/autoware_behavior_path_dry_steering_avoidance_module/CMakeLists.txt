cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_dry_steering_avoidance_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/dry_steering_avoidance_module.cpp
  src/manager.cpp
  src/data_structs.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE config)
