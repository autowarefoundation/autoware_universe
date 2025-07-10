cmake_minimum_required(VERSION 3.14)
project(autoware_default_adapi_universe)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/diagnostics.cpp
  src/fail_safe.cpp
  src/heartbeat.cpp
  src/manual_control.cpp
  src/motion.cpp
  src/mrm_request.cpp
  src/operation_mode.cpp
  src/perception.cpp
  src/planning.cpp
  src/vehicle_status.cpp
  src/vehicle_command.cpp
  src/vehicle_metrics.cpp
  src/vehicle_info.cpp
  src/vehicle_door.cpp
  src/utils/command_conversion.cpp
  src/compatibility/autoware_state.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "autoware::default_adapi::AutowareStateNode"
  "autoware::default_adapi::DiagnosticsNode"
  "autoware::default_adapi::FailSafeNode"
  "autoware::default_adapi::HeartbeatNode"
  "autoware::default_adapi::ManualControlNode"
  "autoware::default_adapi::MotionNode"
  "autoware::default_adapi::MrmRequestNode"
  "autoware::default_adapi::OperationModeNode"
  "autoware::default_adapi::PerceptionNode"
  "autoware::default_adapi::PlanningNode"
  "autoware::default_adapi::VehicleStatusNode"
  "autoware::default_adapi::VehicleCommandNode"
  "autoware::default_adapi::VehicleMetricsNode"
  "autoware::default_adapi::VehicleInfoNode"
  "autoware::default_adapi::VehicleDoorNode"
)

if(BUILD_TESTING)
endif()

install(
  PROGRAMS script/web_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config launch)
