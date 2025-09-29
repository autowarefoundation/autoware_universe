cmake_minimum_required(VERSION 3.14)
project(autoware_steer_offset_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::steer_offset_estimator::SteerOffsetEstimatorNode"
  EXECUTABLE steer_offset_estimator_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_autoware_steer_offset_estimator
    test/test_steer_offset_estimator.cpp
    test/test_utils.cpp
  )
  target_link_libraries(test_autoware_steer_offset_estimator ${PROJECT_NAME})
  ament_target_dependencies(test_autoware_steer_offset_estimator ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endif()

# Install launch files
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
