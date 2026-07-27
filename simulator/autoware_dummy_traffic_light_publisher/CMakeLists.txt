cmake_minimum_required(VERSION 3.14)
project(autoware_dummy_traffic_light_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/traffic_light_cycle.cpp
  src/dummy_traffic_light.cpp
  src/dummy_traffic_light_publisher_node.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_traffic_light_cycle
    test/test_traffic_light_cycle.cpp
  )
  target_include_directories(test_traffic_light_cycle PRIVATE src)
  target_link_libraries(test_traffic_light_cycle ${PROJECT_NAME}_lib)

  ament_add_ros_isolated_gtest(test_dummy_traffic_light
    test/test_dummy_traffic_light.cpp
  )
  target_include_directories(test_dummy_traffic_light PRIVATE src)
  target_link_libraries(test_dummy_traffic_light ${PROJECT_NAME}_lib)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
