cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_modifier)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_trajectory_modifier plugins.xml)

generate_parameter_library(trajectory_modifier_param
  param/trajectory_modifier_parameter_struct.yaml
)

ament_auto_add_library(${PROJECT_NAME}_utils SHARED
  src/trajectory_modifier_utils/utils.cpp
  src/trajectory_modifier_utils/obstacle_stop_utils.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_modifier.cpp
)

target_link_libraries(${PROJECT_NAME}
  trajectory_modifier_param
  ${PROJECT_NAME}_utils
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_modifier::TrajectoryModifier"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  src/trajectory_modifier_plugins/stop_point_fixer.cpp
  src/trajectory_modifier_plugins/obstacle_stop.cpp
)

target_link_libraries(${PROJECT_NAME}_plugins
  ${PROJECT_NAME}
  ${PROJECT_NAME}_utils
  trajectory_modifier_param
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)
  ament_add_ros_isolated_gtest(test_autoware_trajectory_modifier
    ${TEST_SOURCES}
  )
  target_link_libraries(test_autoware_trajectory_modifier
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
    ${PROJECT_NAME}_utils
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
