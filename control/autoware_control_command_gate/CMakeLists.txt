cmake_minimum_required(VERSION 3.8)
project(autoware_control_command_gate)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/control_command_gate.cpp"
  "src/command/interface.cpp"
  "src/command/selector.cpp"
  "src/command/source.cpp"
  "src/command/builtin.cpp"
  "src/command/subscription.cpp"
  "src/command/publisher.cpp"
  "src/command/filter.cpp"
  "src/command/compatibility.cpp"
  "src/command/compatibility/adapi_pause_interface.cpp"
  "src/command/compatibility/emergency_interface.cpp"
  "src/command/compatibility/moderate_stop_interface.cpp"
  "src/common/control_command_filter.cpp"
  "src/common/vehicle_status.cpp"
  "src/common/timeout_diagnostics.cpp"
)
target_include_directories(${PROJECT_NAME} PRIVATE
  "src"
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::control_command_gate::ControlCmdGate"
  EXECUTABLE "control_command_gate_node"
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}_msgs
  "msg/IsFilterActivated.msg"
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(${PROJECT_NAME}_test
    #test/src/test_main.cpp
    "test/src/test_vehicle_cmd_filter.cpp"
    "test/src/test_filter_in_vehicle_cmd_gate_node.cpp"
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    tier4_control_msgs
    ament_index_cpp
  )
  target_include_directories(${PROJECT_NAME}_test PRIVATE src)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
