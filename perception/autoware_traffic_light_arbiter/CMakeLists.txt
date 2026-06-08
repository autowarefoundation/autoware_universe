cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_arbiter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_arbiter.cpp
  src/traffic_light_arbiter_core.cpp
  src/signal_match_validator.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::TrafficLightArbiter"
  EXECUTABLE traffic_light_arbiter_node
)

if(BUILD_TESTING)
  ament_auto_add_gtest(${PROJECT_NAME}_test
    test/test_node.cpp
  )
  ament_auto_add_gtest(${PROJECT_NAME}_characteristic_test
    test/test_traffic_arbiter_characteristic.cpp
  )
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE launch config)
