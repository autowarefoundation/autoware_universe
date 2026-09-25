cmake_minimum_required(VERSION 3.14)
project(autoware_mrm_steering_hold_stop_operator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/deceleration_profile.cpp
  src/steering_hold_stop_logic.cpp
  src/mrm_steering_hold_stop_operator_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::mrm_steering_hold_stop_operator::MrmSteeringHoldStopOperator"
  EXECUTOR "SingleThreadedExecutor"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_deceleration_profile.cpp
    test/test_steering_hold_stop_logic.cpp
    test/test_node_io.cpp
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_component
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
