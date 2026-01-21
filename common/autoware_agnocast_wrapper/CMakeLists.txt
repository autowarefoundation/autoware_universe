cmake_minimum_required(VERSION 3.14)
project(autoware_agnocast_wrapper)

find_package(autoware_cmake REQUIRED)

# Only build on ROS 2 Humble
if(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(WARNING
    "autoware_agnocast_wrapper is only supported on ROS 2 Humble. "
    "Detected ROS_DISTRO='$ENV{ROS_DISTRO}'. Skipping build."
  )

  # Create an empty but valid ament package
  ament_auto_package()
  return()
endif()

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
