cmake_minimum_required(VERSION 3.14)
project(autoware_planning_validator_trajectory_checker)

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

if(BUILD_TESTING)
  find_package(autoware_planning_validator_test_utils REQUIRED)
  ament_add_ros_isolated_gtest(test_autoware_planning_validator
    test/test_main.cpp
    test/test_trajectory_checker.cpp
    test/test_trajectory_checker_diag.cpp
  )
  ament_target_dependencies(test_autoware_planning_validator
    rclcpp
    autoware_planning_msgs
    autoware_planning_validator_test_utils
  )
  target_link_libraries(test_autoware_planning_validator
    ${PROJECT_NAME}
    ${autoware_planning_validator_test_utils_LIBRARIES}
  )
  set_tests_properties(test_autoware_planning_validator PROPERTIES TIMEOUT 120)
endif()

ament_auto_package(INSTALL_TO_SHARE config)
