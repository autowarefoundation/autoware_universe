cmake_minimum_required(VERSION 3.14)
project(component_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_add_library(component_monitor_lib SHARED
  src/component_monitor_core.cpp
)

target_include_directories(component_monitor_lib
  SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(component_monitor_lib
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(component_monitor_lib
  PLUGIN "component_monitor::ComponentMonitorNode"
  EXECUTABLE component_monitor_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
  param
  launch
)
