cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_selector_node.cpp
)

# autoware_trajectory_validator defines rosidl messages, so ament_target_dependencies
# enters "modern cmake" mode and silently ignores _INCLUDE_DIRS and _LIBRARIES for it.
# Supply them directly so the validator's headers and library are properly used.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${autoware_trajectory_validator_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${autoware_trajectory_validator_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_selector::TrajectorySelectorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_auto_find_test_dependencies()

  # ==========================================
  # Dummy Plugin (Test Utility)
  # ==========================================
  pluginlib_export_plugin_description_file(autoware_trajectory_validator
    test/dummy_plugin/dummy_plugins.xml
  )

  ament_auto_add_library(dummy_validator_plugin SHARED
    test/dummy_plugin/dummy_validator_plugin.cpp
  )

  # ==========================================
  # Node Integration Tests
  # ==========================================
  ament_add_ros_isolated_gtest(test_trajectory_selector_node
    test/node/test_trajectory_selector_node.cpp
  )

  target_link_libraries(test_trajectory_selector_node
    ${PROJECT_NAME}
    dummy_validator_plugin
  )

  ament_target_dependencies(test_trajectory_selector_node
    autoware_test_utils
  )
endif()

autoware_ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
