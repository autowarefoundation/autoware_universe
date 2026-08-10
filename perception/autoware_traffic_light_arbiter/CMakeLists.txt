cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_arbiter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_arbiter.cpp
  src/traffic_light_arbiter_node.cpp
  src/signal_match_validator.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::TrafficLightArbiterNode"
  EXECUTABLE traffic_light_arbiter_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING AND NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(${PROJECT_NAME}_integration_test
    test/test_traffic_light_arbiter_integration.cpp
  )
  target_link_libraries(${PROJECT_NAME}_integration_test ${PROJECT_NAME})

  # The core test is ROS-free (no node, no DDS), so it needs no isolation.
  ament_auto_add_gtest(${PROJECT_NAME}_core_test
    test/test_traffic_light_arbiter.cpp
  )
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE launch config)
