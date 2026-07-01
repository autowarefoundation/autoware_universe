cmake_minimum_required(VERSION 3.14)
project(autoware_redundancy_switcher_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/processor.cpp
  src/diag_logic.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/diag_adapter.cpp
  src/log_adapter.cpp
  src/command_mode_subsystem_adapter.cpp
  src/driving_mode_subsystem_adapter.cpp
  src/redundancy_switcher_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
)

target_include_directories(${PROJECT_NAME} PRIVATE
  src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::redundancy_switcher::RedundancySwitcherInterface"
  EXECUTABLE redundancy_switcher_interface_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_processor test/test_processor.cpp)
  target_link_libraries(test_processor ${PROJECT_NAME}_core)

  ament_add_gtest(test_event_gateway test/test_event_gateway.cpp)
  target_link_libraries(test_event_gateway ${PROJECT_NAME}_core)

  ament_add_gtest(test_diag_logic test/test_diag_logic.cpp)
  target_link_libraries(test_diag_logic ${PROJECT_NAME}_core)
  target_include_directories(test_diag_logic PRIVATE src)
  ament_add_gtest(test_command_mode_subsystem_adapter test/test_command_mode_subsystem_adapter.cpp)
  target_link_libraries(test_command_mode_subsystem_adapter ${PROJECT_NAME})
  target_include_directories(test_command_mode_subsystem_adapter PRIVATE src)
endif()

ament_auto_package(INSTALL_TO_SHARE launch config schema)
