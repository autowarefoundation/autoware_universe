cmake_minimum_required(VERSION 3.14)
project(autoware_boundary_departure_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(boundary_departure_checker_utils SHARED
  src/conversion.cpp
  src/steering_abnormality_utils.cpp
  src/utils.cpp
  src/debug.cpp
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
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_gtest(test_${PROJECT_NAME}
    ${TEST_SOURCES}
  )

  target_link_libraries(test_${PROJECT_NAME}
    boundary_departure_checker_utils
    boundary_departure_checker
    uncrossable_boundary_departure_checker
  )
endif()

ament_auto_package()
