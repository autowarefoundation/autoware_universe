cmake_minimum_required(VERSION 3.14)
project(autoware_agnocast_wrapper)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(rclcpp REQUIRED)
find_package(agnocastlib REQUIRED)

include_directories(
  include
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  agnocastlib
)

install(
  FILES cmake/agnocast_wrapper_config_extras.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_auto_package(
  CONFIG_EXTRAS "cmake/agnocast_wrapper_config_extras.cmake"
)
