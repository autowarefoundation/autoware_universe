cmake_minimum_required(VERSION 3.14)
project(autoware_planning_validator_rear_collision_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_planning_validator plugins.xml)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-error=deprecated-declarations)
endif()

include_directories(
  ${autoware_planning_validator_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

generate_parameter_library(rear_collision_checker_node_parameters
  param/rear_collision_checker_node_parameters.yaml
)

target_link_libraries(${PROJECT_NAME}
  ${autoware_planning_validator_LIBRARIES}
  rear_collision_checker_node_parameters
)

ament_auto_package(INSTALL_TO_SHARE config)
