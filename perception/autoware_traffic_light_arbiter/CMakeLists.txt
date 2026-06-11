cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_arbiter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_arbiter.cpp
  src/traffic_light_arbiter_core.cpp
  src/signal_match_validator.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::TrafficLightArbiter"
  EXECUTABLE traffic_light_arbiter_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING AND NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  ament_auto_add_gtest(${PROJECT_NAME}_test
    test/test_node.cpp
  )
  ament_auto_add_gtest(${PROJECT_NAME}_characteristic_test
    test/test_traffic_arbiter_characteristic.cpp
  )
  ament_auto_add_gtest(${PROJECT_NAME}_core_test
    test/test_traffic_light_arbiter_core.cpp
  )
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE launch config)
