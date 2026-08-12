cmake_minimum_required(VERSION 3.14)
project(autoware_obstacle_grid_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/obstacle_grid_utils.cpp
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_obstacle_grid_utils.cpp
  )
endif()

ament_auto_package()
