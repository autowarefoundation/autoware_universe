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
  ament_add_gtest(test_vehicle_constraint_filter
    test/vehicle_constraint_filter.cpp
  )
  target_link_libraries(test_vehicle_constraint_filter
    autoware_trajectory_safety_filter_plugins
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
