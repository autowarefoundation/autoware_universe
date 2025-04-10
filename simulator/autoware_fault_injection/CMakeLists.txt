cmake_minimum_required(VERSION 3.14)
project(autoware_fault_injection)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(pluginlib REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_node_component SHARED
  src/fault_injection_node/fault_injection_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node_component
  PLUGIN "autoware::simulator::fault_injection::FaultInjectionNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  # gtest
  ament_add_ros_isolated_gtest(test_fault_injection_node_component
    test/src/main.cpp
    test/src/test_diagnostic_storage.cpp
  )

  target_link_libraries(test_fault_injection_node_component
    ${PROJECT_NAME}_node_component
  )

  # launch_testing
  add_launch_test(
    test/test_fault_injection_node.test.py
    TARGET test_fault_injection_launch_test
  )
  # TODO(KeisukeShima): Remove in Humble
  set_tests_properties(
    test_fault_injection_launch_test
    PROPERTIES
    LABELS "launch_test"
  )

  install(DIRECTORY
    test/config
    test/launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
