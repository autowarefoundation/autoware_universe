cmake_minimum_required(VERSION 3.5)
project(tier4_control_mode_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)

# include directories
include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
)

# Qt settings
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# create library
add_library(${PROJECT_NAME} SHARED
  src/control_mode_display.cpp
  include/control_mode_display.hpp
)

# link libraries
target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  Qt5::Widgets
  ${pluginlib_LIBRARIES}
)

# include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
)

# add dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  autoware_vehicle_msgs
  pluginlib
)

# install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install plugin description
install(
  DIRECTORY plugins
  DESTINATION share/${PROJECT_NAME}
)

# export dependencies
ament_export_dependencies(
  rclcpp
  rviz_common
  autoware_vehicle_msgs
  Qt5
  pluginlib
)

# export include directories
ament_export_include_directories(include)

# export libraries
ament_export_libraries(${PROJECT_NAME})

# export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# install plugin description
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_package()
