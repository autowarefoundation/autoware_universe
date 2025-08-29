cmake_minimum_required(VERSION 3.14)
project(waypoint_maker)

add_compile_options(-std=c++17 -O3)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_auto_add_executable(waypoint_loader
  src/waypoint_loader/waypoint_loader_core.cpp
)

ament_auto_add_executable(waypoint_saver
  src/waypoint_saver/waypoint_saver.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_waypoint_maker_utils.cpp
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_${PROJECT_NAME}   
    autoware_universe_utils 
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  data
)
