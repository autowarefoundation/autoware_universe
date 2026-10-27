cmake_minimum_required(VERSION 3.14)
project(autoware_mrm_steering_hold_stop_operator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/deceleration_profile.cpp
  src/steering_hold_stop_logic.cpp
  src/mrm_steering_hold_stop_operator_node.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::mrm_steering_hold_stop_operator::MrmSteeringHoldStopOperator"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_deceleration_profile.cpp
    test/test_steering_hold_stop_logic.cpp
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_component
  )

  if(NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    ament_add_ros_isolated_gmock(test_${PROJECT_NAME}_node_io
      test/test_node_io.cpp
    )

    target_include_directories(test_${PROJECT_NAME}_node_io PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    target_link_libraries(test_${PROJECT_NAME}_node_io
      ${PROJECT_NAME}_component
    )
  endif()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
