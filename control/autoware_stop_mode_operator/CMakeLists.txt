cmake_minimum_required(VERSION 3.8)
project(autoware_stop_mode_operator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/stop_mode_operator.cpp"
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::stop_mode_operator::StopModeOperator"
  EXECUTABLE "stop_mode_operator_node"
)

if(BUILD_TESTING)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
