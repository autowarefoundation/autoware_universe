cmake_minimum_required(VERSION 3.14)
project(autoware_pose_covariance_modifier)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_covariance_modifier.cpp
)
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)
if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  find_package(agnocastlib REQUIRED)
  ament_target_dependencies(${PROJECT_NAME} agnocastlib)
endif()
autoware_agnocast_wrapper_setup(${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::pose_covariance_modifier::PoseCovarianceModifierNode"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
