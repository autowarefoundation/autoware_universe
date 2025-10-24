cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_optimizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_trajectory_optimizer_component SHARED
  src/trajectory_optimizer.cpp
  src/utils.cpp
  src/trajectory_optimizer_plugins/trajectory_eb_smoother_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_extender.cpp
  src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_point_fixer.cpp
  src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp
  src/trajectory_optimizer_plugins/trajectory_spline_smoother.cpp
  src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp
)
target_link_libraries(autoware_trajectory_optimizer_component)
rclcpp_components_register_node(autoware_trajectory_optimizer_component
  PLUGIN "autoware::trajectory_optimizer::TrajectoryOptimizer"
  EXECUTABLE autoware_trajectory_optimizer_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)
  ament_add_ros_isolated_gtest(test_autoware_trajectory_optimizer
    ${TEST_SOURCES}
  )
  target_link_libraries(test_autoware_trajectory_optimizer
    autoware_trajectory_optimizer_component
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
