cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_detection_lane_module)

find_package(autoware_cmake REQUIRED)

autoware_package()

find_package(PCL REQUIRED COMPONENTS common)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-error=deprecated-declarations)
endif()

pluginlib_export_plugin_description_file(autoware_behavior_velocity_planner plugins.xml)

generate_parameter_library(detection_lane_parameters
  param/detection_lane_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  detection_lane_parameters
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
#     test/test_utils.cpp
#     test/test_node_interface.cpp
#   )
#   target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
# endif()

ament_auto_package(INSTALL_TO_SHARE config)
