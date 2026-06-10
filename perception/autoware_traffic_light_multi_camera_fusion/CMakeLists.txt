cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_multi_camera_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_multi_camera_fusion_node.cpp
  src/multi_camera_fusion.cpp
  src/traffic_light_multi_camera_fusion_process.cpp
  src/signal_validator.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::MultiCameraFusionNode"
  EXECUTABLE traffic_light_multi_camera_fusion_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING AND (NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  ament_auto_add_gtest(${PROJECT_NAME}_test
    test/test_utils.cpp
    test/test_signal_validator.cpp
    test/test_multi_camera_fusion.cpp
  )
  ament_add_ros_isolated_gtest(${PROJECT_NAME}_node_test
    test/test_traffic_light_multi_camera_fusion_node.cpp
  )
  target_link_libraries(${PROJECT_NAME}_node_test
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
