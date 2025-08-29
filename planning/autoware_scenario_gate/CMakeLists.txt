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

ament_auto_add_library(${PROJECT_NAME}_selectors SHARED
  src/selectors/default_selector.cpp
  src/selectors/extra_selector.cpp
)

pluginlib_export_plugin_description_file(
  autoware_scenario_gate
  plugin_description.xml
)

ament_export_include_directories(include)

# --- Tests ---
if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_gate
    test/test_scenario_gate_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_gate
    ${PROJECT_NAME}
  )

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_default
    test/test_default_selector.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_default
    ${PROJECT_NAME}_selectors
  )

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_extra
    test/test_extra_selector.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_extra
    ${PROJECT_NAME}_selectors
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)