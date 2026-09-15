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
  src/traffic_light_map_based_detector.cpp
  src/traffic_light_map_based_detector_process.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/traffic_light_map_based_detector_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::traffic_light::MapBasedDetector"
  EXECUTABLE traffic_light_map_based_detector_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_utils
    test/test_utils.cpp
  )
  target_link_libraries(test_utils
    ${PROJECT_NAME}
  )
  ament_auto_add_gtest(test_traffic_light_map_based_detector
    test/test_traffic_light_map_based_detector.cpp
  )
  target_link_libraries(test_traffic_light_map_based_detector
    ${PROJECT_NAME}
  )
  ament_auto_add_gtest(test_traffic_light_map_based_detector_node
    test/test_traffic_light_map_based_detector_node.cpp
  )
  target_link_libraries(test_traffic_light_map_based_detector_node
    ${PROJECT_NAME}_node
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
