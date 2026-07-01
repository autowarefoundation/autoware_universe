cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_processor)

find_package(autoware_cmake REQUIRED)

autoware_package()

if(BUILD_TESTING)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_trajectory_processor_integration
    tests/test_trajectory_processor_integration.cpp
  )
  target_compile_options(test_trajectory_processor_integration PRIVATE
    -Wno-error=deprecated-declarations
  )

  ament_target_dependencies(test_trajectory_processor_integration
    autoware_trajectory_modifier
    autoware_trajectory_optimizer
    autoware_test_utils
    rclcpp
    autoware_internal_planning_msgs
    autoware_planning_msgs
    autoware_perception_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
  )
endif()

ament_auto_package()
