cmake_minimum_required(VERSION 3.14)
project(autoware_diagnostic_graph_test_examples)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  get_filename_component(DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data ABSOLUTE)
  file(GLOB_RECURSE test_files "test/*.cpp")
  ament_auto_add_gtest(test_${PROJECT_NAME} ${test_files})
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE TEST_DATA_PATH="${DATA_PATH}")
endif()

ament_auto_package(INSTALL_TO_SHARE data)
