cmake_minimum_required(VERSION 3.8)
project(tier4_manual_lane_change_rviz_plugin)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(autoware_planning_msgs REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

# Define the plugin source
add_library(tier4_manual_lane_change_rviz_plugin SHARED
  src/tier4_manual_lane_change_rviz_plugin.hpp
  src/tier4_manual_lane_change_rviz_plugin.cpp
)

# Link libraries
ament_target_dependencies(tier4_manual_lane_change_rviz_plugin
  rclcpp
  std_msgs
  rviz_common
  autoware_planning_msgs
  tier4_external_api_msgs
  tier4_planning_msgs
)

target_link_libraries(tier4_manual_lane_change_rviz_plugin
  Qt5::Widgets
)

# Export plugin XML
pluginlib_export_plugin_description_file(rviz_common lane_change_plugin.xml)

# Install targets and plugin description
install(TARGETS tier4_manual_lane_change_rviz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES lane_change_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
