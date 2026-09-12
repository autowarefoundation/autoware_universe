cmake_minimum_required(VERSION 3.14)
project(autoware_pose_instability_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_instability_detector.cpp
)
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)
autoware_agnocast_wrapper_setup(${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::pose_instability_detector::PoseInstabilityDetector"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING AND (NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test.cpp
    src/pose_instability_detector.cpp
  )
  ament_target_dependencies(test_${PROJECT_NAME} autoware_agnocast_wrapper)
  autoware_agnocast_wrapper_setup(test_${PROJECT_NAME})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
