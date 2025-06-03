cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_map_based_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_map_based_detector_node.cpp
  src/traffic_light_map_based_detector_process.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::MapBasedDetector"
  EXECUTABLE traffic_light_map_based_detector_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_utils
    test/test_utils.cpp
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
