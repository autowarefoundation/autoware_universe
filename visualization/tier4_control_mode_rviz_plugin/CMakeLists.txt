cmake_minimum_required(VERSION 3.5)
project(tier4_control_mode_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)

include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  src/control_mode_display.cpp
  include/control_mode_display.hpp
)

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  Qt5::Widgets
  ${pluginlib_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  autoware_vehicle_msgs
  pluginlib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY plugins
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rviz_common
  autoware_vehicle_msgs
  Qt5
  pluginlib
)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_package()
