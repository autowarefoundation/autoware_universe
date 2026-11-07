cmake_minimum_required(VERSION 3.14)
project(autoware_speed_scale_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/speed_scale_estimator.cpp
  src/utils.cpp
)

ament_auto_add_library(${PROJECT_NAME}_ros SHARED
  src/speed_scale_estimator_processor.cpp
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}_ros
  PLUGIN "autoware::speed_scale_estimator::SpeedScaleEstimatorNode"
  EXECUTABLE speed_scale_estimator_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_autoware_speed_scale_estimator
    test/test_speed_scale_estimator.cpp
    test/test_utils.cpp
  )
  target_include_directories(test_autoware_speed_scale_estimator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_autoware_speed_scale_estimator ${PROJECT_NAME})
  ament_target_dependencies(test_autoware_speed_scale_estimator ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})

  ament_add_gtest(test_autoware_speed_scale_estimator_processor
    test/test_speed_scale_estimator_processor.cpp
  )
  target_include_directories(
    test_autoware_speed_scale_estimator_processor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(
    test_autoware_speed_scale_estimator_processor ${PROJECT_NAME} ${PROJECT_NAME}_ros)
  ament_target_dependencies(
    test_autoware_speed_scale_estimator_processor ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
