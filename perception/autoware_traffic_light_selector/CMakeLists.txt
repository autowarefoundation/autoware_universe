cmake_minimum_required(VERSION 3.8)
project(autoware_traffic_light_selector)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_selector_node.cpp
  src/traffic_light_selector_utils.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::TrafficLightSelectorNode"
  EXECUTABLE traffic_light_selector_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(${PROJECT_NAME}_test
    test/test_utils.cpp
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
