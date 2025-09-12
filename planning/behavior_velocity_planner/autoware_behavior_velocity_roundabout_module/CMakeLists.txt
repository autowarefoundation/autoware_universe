cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_roundabout_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/manager.cpp
  src/scene_roundabout.cpp
  src/roundabout_lanelets.cpp
  src/object_manager.cpp
  src/decision_result.cpp
  src/scene_roundabout_prepare_data.cpp
  src/debug.cpp
  src/scene_roundabout_collision.cpp
)
ament_auto_package(INSTALL_TO_SHARE config)


install(PROGRAMS
  scripts/ttc.py
  DESTINATION lib/${PROJECT_NAME}
)
