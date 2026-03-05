cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_safety_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_trajectory_safety_filter plugins.xml)

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_safety_filter_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_safety_filter::TrajectorySafetyFilter"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_library(autoware_trajectory_safety_filter_plugins SHARED
  src/filters/out_of_lane_filter.cpp
  src/filters/collision_filter.cpp
  src/filters/vehicle_constraint_filter.cpp
)

target_link_libraries(autoware_trajectory_safety_filter_plugins
  ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_auto_find_test_dependencies()

  # ==========================================
  # 1. Plugin Unit Tests
  # ==========================================
  ament_add_gtest(test_vehicle_constraint_filter
    test/plugins/test_vehicle_constraint_filter.cpp
  )
  target_link_libraries(test_vehicle_constraint_filter
    autoware_trajectory_safety_filter_plugins
  )

  # ==========================================
  # 2. Dummy Plugin (Test Utility)
  # ==========================================
  pluginlib_export_plugin_description_file(autoware_trajectory_safety_filter
    test/dummy_plugin/dummy_plugins.xml
  )

  ament_auto_add_library(dummy_safety_filter_plugin SHARED
    test/dummy_plugin/dummy_safety_filter_plugin.cpp
  )

  target_link_libraries(dummy_safety_filter_plugin
    ${PROJECT_NAME}
  )

  # ==========================================
  # 3. Node Integration/Unit Tests
  # ==========================================
  ament_add_ros_isolated_gtest(test_trajectory_safety_filter_node
    test/node/test_trajectory_safety_filter_node.cpp
  )

  target_link_libraries(test_trajectory_safety_filter_node
    ${PROJECT_NAME}
    dummy_safety_filter_plugin
  )

  ament_target_dependencies(test_trajectory_safety_filter_node
    autoware_test_utils
  )

endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
