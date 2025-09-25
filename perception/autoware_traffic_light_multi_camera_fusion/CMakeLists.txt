cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_multi_camera_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_multi_camera_fusion_node.cpp
  src/traffic_light_multi_camera_fusion_process.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::MultiCameraFusion"
  EXECUTABLE traffic_light_multi_camera_fusion_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(${PROJECT_NAME}_test
    test/test_utils.cpp
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
