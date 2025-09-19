cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_road_user_stop_module)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Generate parameter library
generate_parameter_library(road_user_stop_module_parameters
  param/road_user_stop_module_parameters.yaml
)

pluginlib_export_plugin_description_file(autoware_motion_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# Link parameter library
target_link_libraries(${PROJECT_NAME}
  road_user_stop_module_parameters
)

# Suppress deprecated warnings only for the generated parameter library
target_compile_options(road_user_stop_module_parameters INTERFACE
  -Wno-error=deprecated-declarations
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add utils test
  ament_add_gtest(test_utils
    test/test_utils.cpp
    src/utils.cpp
  )
  target_link_libraries(test_utils
    ${${PROJECT_NAME}_LIBRARIES}
  )
  target_include_directories(test_utils PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  )
endif()

ament_auto_package(INSTALL_TO_SHARE config)
