cmake_minimum_required(VERSION 3.10)
project(autoware_minimap_overlay_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(Qt5 REQUIRED COMPONENTS Network)

set(CMAKE_AUTOMOC ON)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/goal_pose.cpp
  src/minimap.cpp
  src/overlay_utils.cpp
  src/path_overlay.cpp
  src/tile.cpp
  src/tile_field.cpp
  src/tile_provider.cpp
  src/include/goal_pose.hpp
  src/include/minimap.hpp
  src/include/overlay_utils.hpp
  src/include/path_overlay.hpp
  src/include/tile.hpp
  src/include/tile_field.hpp
  src/include/tile_provider.hpp
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Network
  Qt5::Widgets
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
endif()

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(INSTALL_TO_SHARE icons)
