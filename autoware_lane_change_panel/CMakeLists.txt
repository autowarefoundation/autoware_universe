cmake_minimum_required(VERSION 3.8)
project(autoware_lane_change_panel)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Include directories
include_directories(
  include
)

# Define the plugin source
add_library(lane_change_panel SHARED
  include/autoware_lane_change_panel/lane_change_panel.hpp
  src/lane_change_panel.cpp
)

# Link libraries
ament_target_dependencies(lane_change_panel
  rclcpp
  std_msgs
  rviz_common
)

target_link_libraries(lane_change_panel
  Qt5::Widgets
)

# Export plugin XML
pluginlib_export_plugin_description_file(rviz_common lane_change_plugin.xml)

# Install targets and plugin description
install(TARGETS lane_change_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES lane_change_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install header files (if needed)
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
