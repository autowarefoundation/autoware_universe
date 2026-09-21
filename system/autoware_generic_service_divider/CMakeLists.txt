cmake_minimum_required(VERSION 3.14)
project(autoware_generic_service_divider)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Core library: plugin base (the generic service/client themselves now come from
# autoware_agnocast_wrapper, which also gives this the runtime rclcpp/Agnocast switch)
ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/service_divider_plugin_base.cpp
)

# Plugin library: concrete divider plugins
ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  plugins/change_operation_mode_divider.cpp
  plugins/change_autoware_control_divider.cpp
  plugins/control_mode_request_divider.cpp
  plugins/reset_diag_graph_divider.cpp
  plugins/reset_redundancy_switcher_divider.cpp
  plugins/set_aggregator_initializing_divider.cpp
  plugins/set_redundancy_switcher_interface_initializing_divider.cpp
  plugins/ekf_trigger_node_divider.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME}_core)

# Node component
ament_auto_add_library(${PROJECT_NAME}_node_component SHARED
  src/generic_service_divider_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node_component ${PROJECT_NAME}_core)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}_node_component
  PLUGIN "generic_service_divider::GenericServiceDividerNode"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR MultiThreadedAgnocastExecutor
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_generic_service_divider
    test/test_generic_service_divider.cpp
  )
  target_link_libraries(test_generic_service_divider
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_plugins
  )
  ament_target_dependencies(test_generic_service_divider
    autoware_agnocast_wrapper
    rclcpp
    pluginlib
    autoware_system_msgs
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
