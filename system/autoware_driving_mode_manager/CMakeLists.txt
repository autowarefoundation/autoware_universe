cmake_minimum_required(VERSION 3.8)
project(autoware_driving_mode_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/DebugModeRequest.msg"
  "msg/DebugModeFlag.msg"
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(
  rosidl_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

ament_auto_add_library(${PROJECT_NAME}_common SHARED
  "src/core/config.cpp"
  "src/core/status.cpp"
  "src/core/task.cpp"
  "src/core/values.cpp"
  "src/core/init.cpp"
  "src/core/manager.cpp"
  "src/ros_interface.cpp"
  "src/driving_mode_manager.cpp"
)
target_include_directories(${PROJECT_NAME}_common PRIVATE
  "src"
)
target_link_libraries(${PROJECT_NAME}_common
  "${rosidl_typesupport_target}"
)

ament_auto_add_library(${PROJECT_NAME}_plugin SHARED
  "src/plugin/default.cpp"
)
target_include_directories(${PROJECT_NAME}_plugin PRIVATE
  "src"
)

rclcpp_components_register_node(${PROJECT_NAME}_common
  PLUGIN "autoware::driving_mode_manager::DrivingModeManager"
  EXECUTABLE "driving_mode_manager_node"
)

if(BUILD_TESTING)
endif()

pluginlib_export_plugin_description_file(autoware_driving_mode_manager plugins.xml)
ament_auto_package(INSTALL_TO_SHARE config launch)
