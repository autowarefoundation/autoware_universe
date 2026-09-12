cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(trajectory_adapter_lib_target ${PROJECT_NAME}_lib)

ament_auto_add_library(${trajectory_adapter_lib_target} SHARED
  src/trajectory_adapter.cpp
  src/trajectory_adapter_wrapper.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
)
