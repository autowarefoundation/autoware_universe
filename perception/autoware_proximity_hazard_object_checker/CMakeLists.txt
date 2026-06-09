cmake_minimum_required(VERSION 3.14)
project(autoware_proximity_hazard_object_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/proximity_hazard_object_checker.cpp
  src/proximity_hazard_object_checker_node.cpp
)

target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_param
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::proximity_hazard_object_checker::ProximityHazardObjectCheckerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
