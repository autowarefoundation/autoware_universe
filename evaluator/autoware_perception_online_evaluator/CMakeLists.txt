cmake_minimum_required(VERSION 3.14)
project(autoware_perception_online_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(pluginlib REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/metrics_calculator.cpp
  src/perception_online_evaluator_node.cpp
  src/metrics/deviation_metrics.cpp
  src/metrics/detection_count.cpp
  src/utils/marker_utils.cpp
  src/utils/objects_filtering.cpp
  src/perception_analytics_calculator.cpp
  src/perception_analytics_publisher_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN   "autoware::perception_diagnostics::PerceptionOnlineEvaluatorNode"
  EXECUTABLE autoware_perception_online_evaluator_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN   "autoware::perception_diagnostics::PerceptionAnalyticsPublisherNode"
  EXECUTABLE autoware_perception_analytics_publisher_node
)

target_link_libraries(${PROJECT_NAME} glog::glog)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_perception_online_evaluator_node
    test/test_perception_online_evaluator_node.cpp
    TIMEOUT 300
  )
  target_link_libraries(test_perception_online_evaluator_node
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    param
    launch
)
