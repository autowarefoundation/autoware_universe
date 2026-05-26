cmake_minimum_required(VERSION 3.8)
project(autoware_trajectory_gate)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/trajectory_gate.cpp"
  "src/core/interface.cpp"
  "src/core/monitor.cpp"
  "src/core/selector.cpp"
  "src/core/subscription.cpp"
  "src/core/publisher.cpp"
)
target_include_directories(${PROJECT_NAME} PRIVATE "src")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_gate::TrajectoryGate"
  EXECUTABLE "trajectory_gate_node"
)

if(BUILD_TESTING)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
