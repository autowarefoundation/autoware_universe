cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_obstacle_slow_down_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_motion_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_planning_factor.cpp
  )

  target_link_libraries(
    test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(INSTALL_TO_SHARE config)
