cmake_minimum_required(VERSION 3.14)
project(tier4_perception_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(OD_PLUGIN_LIB_SRC
  src/object_detection/detected_objects_wf_display.cpp
  src/object_detection/detected_objects_wf_helper.cpp
)

set(OD_PLUGIN_LIB_HEADERS
  include/tier4_perception_rviz_plugin/visibility_control.hpp
)

set(OD_PLUGIN_LIB_HEADERS_TO_WRAP
  include/tier4_perception_rviz_plugin/object_detection/detected_objects_wf_display.hpp
  include/tier4_perception_rviz_plugin/object_detection/detected_objects_wf_helper.hpp
)

foreach(header "${OD_PLUGIN_LIB_HEADERS_TO_WRAP}")
  qt5_wrap_cpp(OD_PLUGIN_LIB_HEADERS_MOC "${header}")
endforeach()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${COMMON_SRC}
  ${OD_PLUGIN_LIB_HEADERS}
  ${OD_PLUGIN_LIB_HEADERS_MOC}
  ${OD_PLUGIN_LIB_SRC}
)
target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  Qt5::Widgets
)
target_include_directories(${PROJECT_NAME} PRIVATE include)

# Settings to improve the build as suggested on https://github.com/ros2/rviz/blob/ros2/docs/plugin_development.md
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "OBJECT_DETECTION_PLUGINS_BUILDING_LIBRARY")

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  icons
)
