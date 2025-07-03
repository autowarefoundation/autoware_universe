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
  "src/common/control_command_filter.cpp"
  "src/common/vehicle_status.cpp"
  "src/common/timeout_diagnostics.cpp"
)
target_include_directories(${PROJECT_NAME} PRIVATE "src")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::control_command_gate::ControlCmdGate"
  EXECUTABLE "control_command_gate_node"
)

if(BUILD_TESTING)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
