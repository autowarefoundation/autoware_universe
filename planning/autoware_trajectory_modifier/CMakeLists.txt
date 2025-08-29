cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_modifier)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_trajectory_modifier_component SHARED
  src/trajectory_modifier.cpp
  src/utils.cpp
  src/trajectory_modifier_plugins/stop_point_fixer.cpp
)

target_link_libraries(autoware_trajectory_modifier_component)

rclcpp_components_register_node(autoware_trajectory_modifier_component
  PLUGIN "autoware::trajectory_modifier::TrajectoryModifier"
  EXECUTABLE autoware_trajectory_modifier_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)
  ament_add_ros_isolated_gtest(test_autoware_trajectory_modifier
    ${TEST_SOURCES}
  )
  target_link_libraries(test_autoware_trajectory_modifier
    autoware_trajectory_modifier_component
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
