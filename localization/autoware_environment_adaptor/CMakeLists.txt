cmake_minimum_required(VERSION 3.14)
project(autoware_environment_adaptor)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(LIBRARY_NAME ${PROJECT_NAME}_lib)
ament_auto_add_library(${LIBRARY_NAME} SHARED
  src/environment_classifier.cpp
  src/environment_adaptor.cpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
target_link_libraries(${LIBRARY_NAME} Eigen3::Eigen)

rclcpp_components_register_node(${LIBRARY_NAME}
  PLUGIN "autoware::environment_adaptor::EnvironmentAdaptor"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_environment_classifier
    test/test_environment_classifier.cpp
  )
  target_include_directories(test_environment_classifier PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_environment_classifier ${LIBRARY_NAME})
  ament_target_dependencies(test_environment_classifier
    autoware_lanelet2_utils
    autoware_map_msgs
    geometry_msgs
  )

  ament_auto_add_gtest(test_environment_adaptor
    test/test_environment_adaptor.cpp
  )
  target_include_directories(test_environment_adaptor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_environment_adaptor ${LIBRARY_NAME})
  ament_target_dependencies(test_environment_adaptor
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
