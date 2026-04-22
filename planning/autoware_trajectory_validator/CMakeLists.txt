cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_trajectory_validator plugins.xml)

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

set(trajectory_validator_lib_target ${PROJECT_NAME}_lib)

include_directories(
  SYSTEM
)

ament_auto_add_library(${trajectory_validator_lib_target} SHARED
  src/trajectory_validator_node.cpp
)

target_link_libraries(${trajectory_validator_lib_target}
  ${PROJECT_NAME}_param
)

target_compile_options(${trajectory_validator_lib_target} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${trajectory_validator_lib_target}
  PLUGIN "autoware::trajectory_validator::TrajectoryValidator"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_library(autoware_trajectory_validator_plugins SHARED
  src/filters/safety/out_of_lane_filter.cpp
  src/filters/safety/collision_filter.cpp
  src/filters/safety/vehicle_constraint_filter.cpp
  src/filters/traffic_rule/traffic_light_filter.cpp
)

target_link_libraries(autoware_trajectory_validator_plugins
  ${trajectory_validator_lib_target}
)

# Clear stale Python interpreter state so rosidl can resolve it correctly.
unset(PYTHON_EXECUTABLE CACHE)
unset(PYTHON_EXECUTABLE)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MetricReport.msg"
  "msg/ValidationReport.msg"
  "msg/ValidationReportArray.msg"
  DEPENDENCIES
  builtin_interfaces
  unique_identifier_msgs
)

# to use messages defined in the same package from C++
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(${trajectory_validator_lib_target}
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${trajectory_validator_lib_target} "${cpp_typesupport_target}")
endif()

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
    autoware_trajectory_validator_plugins
  )

  # ==========================================
  # 2. Dummy Plugin (Test Utility)
  # ==========================================
  pluginlib_export_plugin_description_file(autoware_trajectory_validator
    test/dummy_plugin/dummy_plugins.xml
  )

  ament_auto_add_library(dummy_validator_plugin SHARED
    test/dummy_plugin/dummy_validator_plugin.cpp
  )

  target_link_libraries(dummy_validator_plugin
    ${trajectory_validator_lib_target}
  )

  # ==========================================
  # 3. Node Integration/Unit Tests
  # ==========================================
  ament_add_ros_isolated_gtest(test_trajectory_validator_node
    test/node/test_trajectory_validator_node.cpp
  )

  target_link_libraries(test_trajectory_validator_node
    ${trajectory_validator_lib_target}
    dummy_validator_plugin
  )

  ament_target_dependencies(test_trajectory_validator_node
    autoware_test_utils
  )

  ament_add_gtest(test_traffic_light_filter
    test/plugins/test_traffic_light_filter.cpp
  )
  target_link_libraries(test_traffic_light_filter
    autoware_trajectory_validator_plugins
  )


endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
