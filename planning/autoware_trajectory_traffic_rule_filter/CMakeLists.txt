cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_traffic_rule_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_trajectory_traffic_rule_filter plugins.xml)

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_traffic_rule_filter_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_traffic_rule_filter::TrajectoryTrafficRuleFilter"
  EXECUTABLE ${PROJECT_NAME}_node
)

# Add plugin library
ament_auto_add_library(autoware_trajectory_traffic_rule_filter_plugins SHARED
  src/filters/traffic_light_filter.cpp
)

target_link_libraries(autoware_trajectory_traffic_rule_filter_plugins
  ${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
