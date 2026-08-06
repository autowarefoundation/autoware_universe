cmake_minimum_required(VERSION 3.14)
project(autoware_obstacle_grid_extractor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  lib/obstacle_grid_extractor.cpp
  src/obstacle_grid_extractor_node.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE src)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::obstacle_grid_extractor::ObstacleGridExtractorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_obstacle_grid_extractor.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  ament_auto_add_gtest(test_grid_coverage
    test/test_grid_coverage.cpp
  )
  target_link_libraries(test_grid_coverage
    ${PROJECT_NAME}
  )

  ament_auto_add_gtest(test_${PROJECT_NAME}_node
    test/test_obstacle_grid_extractor_node.cpp
  )
  target_include_directories(test_${PROJECT_NAME}_node PRIVATE src)
  target_link_libraries(test_${PROJECT_NAME}_node
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  schema
)
