cmake_minimum_required(VERSION 3.14)
project(autoware_covariance_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(LIBRARY_NAME ${PROJECT_NAME}_lib)
ament_auto_add_library(${LIBRARY_NAME} SHARED
  src/covariance_selector.cpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

find_package(Boost REQUIRED)

rclcpp_components_register_node(${LIBRARY_NAME}
  PLUGIN "autoware::covariance_selector::CovarianceSelector"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
