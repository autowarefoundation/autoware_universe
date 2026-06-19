cmake_minimum_required(VERSION 3.14)
project(autoware_velocity_scale_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(LIBRARY_NAME ${PROJECT_NAME}_lib)
ament_auto_add_library(${LIBRARY_NAME} SHARED
  src/velocity_scale_selector.cpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

find_package(Boost REQUIRED)

rclcpp_components_register_node(${LIBRARY_NAME}
  PLUGIN "autoware::velocity_scale_selector::VelocityScaleSelector"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_velocity_scale_selector.cpp
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_${PROJECT_NAME} ${LIBRARY_NAME})
  ament_target_dependencies(test_${PROJECT_NAME}
    autoware_lanelet2_utils
    autoware_map_msgs
    geometry_msgs
    rclcpp
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
