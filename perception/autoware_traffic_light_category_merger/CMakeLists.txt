cmake_minimum_required(VERSION 3.8)
project(autoware_traffic_light_category_merger)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_category_merger.cpp
  src/traffic_light_category_merger_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::TrafficLightCategoryMergerNode"
  EXECUTABLE traffic_light_category_merger_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Node-level integration tests (topic-driven pub/sub + sync).
  ament_add_ros_isolated_gtest(test_traffic_light_category_merger_integration
    test/test_traffic_light_category_merger_integration.cpp
  )
  target_link_libraries(test_traffic_light_category_merger_integration
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
