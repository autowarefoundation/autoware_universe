cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_obstacle_slow_down_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_motion_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

if(BUILD_TESTING)
  # Add test executable
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_planning_factor.cpp
  )

  # Set test dependencies
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
  )

  # Set test include directories
  target_include_directories(test_${PROJECT_NAME}
    PRIVATE
    src
  )
endif()


ament_auto_package(INSTALL_TO_SHARE config)
