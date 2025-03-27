cmake_minimum_required(VERSION 3.14)
project(waypoint_maker_rviz_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/include/waypoint_saver_panel.hpp
  src/waypoint_saver_panel.cpp
  src/include/waypoint_loader_panel.hpp
  src/waypoint_loader_panel.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)


ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
