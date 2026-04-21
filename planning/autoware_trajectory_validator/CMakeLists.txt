cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_validator_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_validator::TrajectoryValidator"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  src/filters/safety/out_of_lane_filter.cpp
  src/filters/safety/collision_filter.cpp
  src/filters/safety/vehicle_constraint_filter.cpp
  src/filters/traffic_rule/traffic_light_filter.cpp
)

target_link_libraries(${PROJECT_NAME}_plugins
  ${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()

  pluginlib_export_plugin_description_file(${PROJECT_NAME}
    test/dummy_plugin/dummy_plugins.xml
  )

  ament_auto_add_library(dummy_validator_plugin SHARED
    test/dummy_plugin/dummy_validator_plugin.cpp
  )

  target_link_libraries(dummy_validator_plugin
    ${PROJECT_NAME}
  )

  set(test_sources
    test/node/test_trajectory_validator_node.cpp
    test/plugins/test_traffic_light_filter.cpp
    test/plugins/test_vehicle_constraint_filter.cpp
  )

  set(RUNNER "RUNNER" "${ament_cmake_ros_DIR}/run_test_isolated.py")

  ament_auto_add_gtest(test_${PROJECT_NAME}
    ${test_sources}
    RUNNER "${RUNNER}"
  )

endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
