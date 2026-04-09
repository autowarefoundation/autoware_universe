cmake_minimum_required(VERSION 3.14)
project(autoware_dynamic_lanelet_map_provider)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(lanelet2_map_provider_node SHARED
  src/lanelet2_map_provider_node.cpp
)

rclcpp_components_register_node(lanelet2_map_provider_node
  PLUGIN "autoware::dynamic_lanelet_map_provider::Lanelet2MapProviderNode"
  EXECUTABLE autoware_lanelet2_map_provider
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})
    ament_add_gmock(${test_name} ${filepath})
    target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(${test_name} ${${PROJECT_NAME}_LIBRARIES})
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()

  add_testcase(test/test_lanelet2_map_provider_node.cpp)
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
