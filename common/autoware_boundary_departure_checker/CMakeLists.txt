cmake_minimum_required(VERSION 3.14)
project(autoware_boundary_departure_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(boundary_departure_checker_utils SHARED
  src/conversion.cpp
  src/steering_abnormality_utils.cpp
  src/utils.cpp
  src/debug.cpp
  src/footprint_generator/normal_footprint.cpp
  src/footprint_generator/localization_footprint.cpp
  src/footprint_generator/longitudinal_footprint.cpp
  src/footprint_generator/steering_footprint.cpp
  src/footprint_generator/footprint_manager.cpp
)

ament_auto_add_library(uncrossable_boundary_departure_checker SHARED
  src/uncrossable_boundary_departure_checker.cpp
)

target_link_libraries(uncrossable_boundary_departure_checker
  boundary_departure_checker_utils
)

ament_auto_add_library(boundary_departure_checker SHARED
  src/boundary_departure_checker.cpp
)

target_link_libraries(boundary_departure_checker
  boundary_departure_checker_utils
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(python_cmake_module REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  find_package(pybind11_vendor REQUIRED)
  find_package(pybind11 REQUIRED)
  option(EXPORT_TEST_PLOT_FIGURE "Enable plotting in tests" OFF)
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_gtest(test_${PROJECT_NAME}
    ${TEST_SOURCES}
  )
  target_link_libraries(test_${PROJECT_NAME}
    boundary_departure_checker_utils
    boundary_departure_checker
    uncrossable_boundary_departure_checker
    Python3::Python
    pybind11::embed
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  )
  ament_target_dependencies(test_${PROJECT_NAME} autoware_pyplot)
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE
    TEST_PACKAGE_NAME="${PROJECT_NAME}"
  )
  if(EXPORT_TEST_PLOT_FIGURE)
    message(STATUS "EXPORT_TEST_PLOT_FIGURE is ON")
    target_compile_definitions(test_${PROJECT_NAME} PRIVATE
      EXPORT_TEST_PLOT_FIGURE
    )
  else()
    message(STATUS "EXPORT_TEST_PLOT_FIGURE is OFF")
  endif()
endif()

ament_auto_package()
