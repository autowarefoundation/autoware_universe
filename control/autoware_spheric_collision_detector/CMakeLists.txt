cmake_minimum_required(VERSION 3.14)
project(autoware_spheric_collision_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(spheric_collision_detector SHARED
  src/spheric_collision_detector_node/sphere3.cpp
  src/spheric_collision_detector_node/spheric_collision_detector.cpp
  src/spheric_collision_detector_node/spheric_collision_detector_node.cpp
)

rclcpp_components_register_node(spheric_collision_detector
  PLUGIN "spheric_collision_detector::SphericCollisionDetectorNode"
  EXECUTABLE spheric_collision_detector_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_spheric_collision_detector
    test/test_spheric_collision_detector.cpp
  )
  target_link_libraries(test_spheric_collision_detector spheric_collision_detector)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
