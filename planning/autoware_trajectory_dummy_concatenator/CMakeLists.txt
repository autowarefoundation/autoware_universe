cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_dummy_concatenator)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_dummy_concatenator_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_dummy_concatenator::TrajectoryDummyConcatenatorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
