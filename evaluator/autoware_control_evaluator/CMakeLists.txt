cmake_minimum_required(VERSION 3.14)
project(autoware_control_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(pluginlib REQUIRED)

ament_auto_add_library(control_evaluator_node SHARED
  DIRECTORY src
)

rclcpp_components_register_node(control_evaluator_node
  PLUGIN "control_diagnostics::ControlEvaluatorNode"
  EXECUTABLE control_evaluator
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_control_evaluator
    test/test_control_evaluator_node.cpp
  )
  target_link_libraries(test_control_evaluator
    control_evaluator_node
  )
endif()


ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
