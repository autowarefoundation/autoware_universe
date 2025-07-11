cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_rviz_plugin)

find_package(ament_cmake_auto REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(autoware_lanelet2_extension REQUIRED)
find_package(Qt5 REQUIRED Core Widgets)
find_package(rosidl_default_runtime REQUIRED)

set(QT_LIBRARIES Qt5::Widgets)

# Enable Qt MOC and include current dir for Qt headers
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

# Recursively collect all .cpp and .hpp files
file(GLOB_RECURSE SRC_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
file(GLOB_RECURSE HDR_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/include/autoware_traffic_light_rviz_plugin/*.hpp"
)

# Create plugin library from all sources
ament_auto_add_library(autoware_traffic_light_rviz_plugin SHARED
  ${SRC_FILES}
  ${HDR_FILES}
)

ament_target_dependencies(autoware_traffic_light_rviz_plugin
  rclcpp
  tf2_ros
  pluginlib
  rviz_common
  ament_index_cpp
  autoware_perception_msgs
  rosidl_default_runtime
  autoware_lanelet2_extension
)

target_link_libraries(autoware_traffic_light_rviz_plugin
  ${QT_LIBRARIES}
)

# Suppress specific warnings if desired
target_compile_options(autoware_traffic_light_rviz_plugin PUBLIC
  -Wno-error=deprecated-copy
  -Wno-error=pedantic
)

# Export plugin description for RViz
pluginlib_export_plugin_description_file(
  rviz_common
  plugins/plugin_description.xml
)

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)

include_directories(${rviz_common_INCLUDE_DIRS})
