cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_compliance_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_compliance_checker.cpp
  src/traffic_light_status_tracker.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
