cmake_minimum_required(VERSION 3.14)
project(autoware_scenario_gate)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scenario_gate.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::scenario_gate::ScenarioGateNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_export_include_directories(include)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)