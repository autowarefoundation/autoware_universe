cmake_minimum_required(VERSION 3.8)
project(autoware_traffic_light_selector)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_selector.cpp
  src/traffic_light_selector_utils.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/traffic_light_selector_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::traffic_light::TrafficLightSelectorNode"
  EXECUTABLE traffic_light_selector_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(${PROJECT_NAME}_test
    test/test_utils.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )
  ament_auto_add_gtest(${PROJECT_NAME}_selector_test
    test/test_traffic_light_selector.cpp
  )
  target_link_libraries(${PROJECT_NAME}_selector_test
    ${PROJECT_NAME}
  )
  ament_auto_add_gtest(${PROJECT_NAME}_node_test
    test/test_traffic_light_selector_node.cpp
  )
  target_link_libraries(${PROJECT_NAME}_node_test
    ${PROJECT_NAME}_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
