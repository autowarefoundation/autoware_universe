cmake_minimum_required(VERSION 3.14)
project(autoware_redundancy_switcher_interface_plugins)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(nlohmann_json REQUIRED)

pluginlib_export_plugin_description_file(autoware_redundancy_switcher_interface plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/redundancy_switcher/redundancy_switcher_adapter.cpp
  src/non_redundant/non_redundant_adapter.cpp
  src/simple_switcher/switcher_adapter.cpp
)

ament_auto_add_library(simple_switcher_node_component SHARED
  src/simple_switcher/simple_switcher_node.cpp
)

rclcpp_components_register_node(simple_switcher_node_component
  PLUGIN "autoware::redundancy_switcher::SimpleSwitcherNode"
  EXECUTABLE simple_switcher_node
)

target_include_directories(${PROJECT_NAME} PRIVATE
  src/redundancy_switcher
  src/non_redundant
  src/simple_switcher)
target_include_directories(simple_switcher_node_component PRIVATE src/simple_switcher)
target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_redundancy_switcher_adapter test/test_redundancy_switcher_adapter.cpp)
  target_link_libraries(test_redundancy_switcher_adapter ${PROJECT_NAME})
  target_include_directories(test_redundancy_switcher_adapter PRIVATE src/redundancy_switcher)
  ament_add_gtest(test_non_redundant_adapter test/test_non_redundant_adapter.cpp)
  target_link_libraries(test_non_redundant_adapter ${PROJECT_NAME})
  target_include_directories(test_non_redundant_adapter PRIVATE src/non_redundant)
  ament_add_gtest(test_simple_switcher_adapter test/test_simple_switcher_adapter.cpp)
  target_link_libraries(test_simple_switcher_adapter ${PROJECT_NAME})
  target_include_directories(test_simple_switcher_adapter PRIVATE src/simple_switcher)
  ament_add_gtest(test_simple_switcher_node test/test_simple_switcher_node.cpp)
  target_link_libraries(test_simple_switcher_node simple_switcher_node_component)
  target_include_directories(test_simple_switcher_node PRIVATE src/simple_switcher)
endif()

ament_auto_package(INSTALL_TO_SHARE config docs launch)
