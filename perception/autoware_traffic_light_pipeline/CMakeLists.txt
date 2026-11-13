cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_pipeline)

# The composed cores are only built when CUDA / TensorRT are available: autoware_tensorrt_yolox
# returns early (installing no headers) when autoware_tensorrt_common is missing, so this package
# cannot be compiled at all on such an environment. Skip it the same way instead of failing.
find_package(autoware_tensorrt_yolox)
if(NOT autoware_tensorrt_yolox_FOUND)
  message(WARNING "The autoware_tensorrt_yolox package is not found. Please check its dependencies.")
  return()
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

# ROS-free composed core
ament_auto_add_library(autoware_traffic_light_recognition SHARED
  src/traffic_light_recognition/traffic_light_recognition.cpp
)

# Node adapter (including the parameter -> config mapping)
ament_auto_add_library(autoware_traffic_light_recognition_node SHARED
  src/traffic_light_recognition/traffic_light_recognition_node.cpp
)
target_link_libraries(autoware_traffic_light_recognition_node autoware_traffic_light_recognition)

rclcpp_components_register_node(autoware_traffic_light_recognition_node
  PLUGIN "autoware::traffic_light::TrafficLightRecognitionNode"
  EXECUTABLE traffic_light_recognition_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_ros_isolated_gtest(test_traffic_light_recognition_node
    test/test_traffic_light_recognition_node.cpp
    TIMEOUT 600
  )
  target_include_directories(test_traffic_light_recognition_node PRIVATE src)
  target_link_libraries(test_traffic_light_recognition_node
    autoware_traffic_light_recognition_node)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
