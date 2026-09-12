cmake_minimum_required(VERSION 3.14)
project(autoware_diagnostic_graph_merger)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/diagnostic_graph_merger/merge.cpp"
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::diagnostic_graph_merger::MergeNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE config launch)
