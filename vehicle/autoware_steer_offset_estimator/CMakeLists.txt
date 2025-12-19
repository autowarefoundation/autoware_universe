cmake_minimum_required(VERSION 3.5)
project(autoware_steer_offset_estimator)

find_package(autoware_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
autoware_package()

ament_auto_add_library(autoware_steer_offset_estimator_lib SHARED
  src/steer_offset_estimator.cpp
  src/node.cpp
  src/utils.cpp
)

# as a ros2 node
ament_auto_add_executable(autoware_steer_offset_estimator src/main.cpp)
target_link_libraries(autoware_steer_offset_estimator
  autoware_steer_offset_estimator_lib
  ${glog_LIBRARIES}
)
target_include_directories(autoware_steer_offset_estimator PRIVATE ${glog_INCLUDE_DIRS})

# as a component
rclcpp_components_register_node(autoware_steer_offset_estimator_lib
  PLUGIN "autoware::steer_offset_estimator::SteerOffsetEstimatorNode"
  EXECUTABLE steer_offset_estimator_component
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_autoware_steer_offset_estimator
    test/test_steer_offset_estimator.cpp
    test/test_utils.cpp
  )
  target_link_libraries(test_autoware_steer_offset_estimator autoware_steer_offset_estimator_lib)
  ament_target_dependencies(test_autoware_steer_offset_estimator ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
