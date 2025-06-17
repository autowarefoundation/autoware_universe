cmake_minimum_required(VERSION 3.22)
project(autoware_planning_validator_test_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

ament_auto_package(INSTALL_TO_SHARE)
