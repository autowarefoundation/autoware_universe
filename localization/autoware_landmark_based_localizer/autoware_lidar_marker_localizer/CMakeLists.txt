cmake_minimum_required(VERSION 3.14)
project(autoware_lidar_marker_localizer)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lidar_marker_localizer.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lidar_marker_localizer::LidarMarkerLocalizer"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lidar_marker_localizer test/test_lidar_marker_localizer.cpp)
  target_include_directories(test_lidar_marker_localizer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_lidar_marker_localizer
    ${PROJECT_NAME}
    ${PCL_LIBRARIES}
  )
  ament_target_dependencies(test_lidar_marker_localizer rclcpp rclcpp_components)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
