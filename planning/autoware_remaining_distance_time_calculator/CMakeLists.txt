cmake_minimum_required(VERSION 3.14)
project(autoware_remaining_distance_time_calculator)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(remaining_distance_time_calculator_parameters
  param/remaining_distance_time_calculator_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/remaining_distance_time_calculator_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  remaining_distance_time_calculator_parameters
)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::remaining_distance_time_calculator::RemainingDistanceTimeCalculatorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)


ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
