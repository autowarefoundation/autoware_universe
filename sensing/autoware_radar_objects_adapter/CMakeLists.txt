cmake_minimum_required(VERSION 3.14)
project(autoware_radar_objects_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/radar_objects_adapter.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::radar_objects_adapter::RadarObjectsAdapterNode"
  EXECUTABLE radar_objects_adapter_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  # ROS-based tests can only run while agnocast is disabled: the node's pub/sub then go through rclcpp.
  if(NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    # Characterization test: drives RadarObjectsAdapterNode over its real topics. Runs in an isolated
    # ROS domain so that it cannot interfere with other tests.
    ament_add_ros_isolated_gtest(test_radar_objects_adapter_characterization
      test/test_radar_objects_adapter_characterization.cpp
      TIMEOUT 300
    )
    target_include_directories(test_radar_objects_adapter_characterization PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_radar_objects_adapter_characterization
      ${PROJECT_NAME}
    )
    ament_target_dependencies(test_radar_objects_adapter_characterization
      autoware_agnocast_wrapper
      autoware_perception_msgs
      autoware_sensing_msgs
      rclcpp
    )
  endif()
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
