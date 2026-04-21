cmake_minimum_required(VERSION 3.14)
project(autoware_boundary_departure_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(utils SHARED
  src/conversion.cpp
  src/geometry_projection.cpp
  src/uncrossable_boundaries_rtree.cpp
  src/boundary_segment_finder.cpp
  src/severity_evaluator.cpp
  src/hysteresis_logic.cpp
  src/boundary_departure_evaluator.cpp
  src/footprints_generator.cpp
  src/debug.cpp
)

ament_auto_add_library(uncrossable_boundary_checker SHARED
  src/uncrossable_boundary_checker.cpp
)

target_link_libraries(uncrossable_boundary_checker
  utils
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_gtest(test_${PROJECT_NAME}
    ${TEST_SOURCES}
  )
  target_link_libraries(test_${PROJECT_NAME}
    utils
    uncrossable_boundary_checker
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
  )
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE
    TEST_PACKAGE_NAME="${PROJECT_NAME}"
  )
endif()

ament_auto_package()
