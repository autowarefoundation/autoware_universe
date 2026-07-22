cmake_minimum_required(VERSION 3.14)
project(autoware_external_cmd_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(diagnostic_updater REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/autoware_external_cmd_selector/external_cmd_selector_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${diagnostic_updater_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::external_cmd_selector::ExternalCmdSelector"
  EXECUTABLE autoware_external_cmd_selector
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
