cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_ranker)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_trajectory_ranker plugins.xml)

generate_parameter_library(trajectory_ranker_parameters
  param/trajectory_ranker_parameters.yaml
)

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_ranker_node.cpp
  src/evaluation.cpp
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  trajectory_ranker_parameters
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_ranker::TrajectoryRanker"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_library(autoware_trajectory_ranker_plugins SHARED
  src/metrics/lateral_acceleration_metric.cpp
  src/metrics/longitudinal_jerk_metric.cpp
  src/metrics/time_to_collision_metric.cpp
  src/metrics/distance_metric.cpp
  src/metrics/lateral_deviation_metric.cpp
  src/metrics/steering_consistency_metric.cpp
  src/metrics/metrics_utils.cpp
)

target_link_libraries(autoware_trajectory_ranker_plugins
  ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_utils
    test/test_utils.cpp
  )
  target_link_libraries(test_utils
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_metrics_utils
    test/test_metrics_utils.cpp
  )
  target_link_libraries(test_metrics_utils
    autoware_trajectory_ranker_plugins
  )

  ament_add_gtest(test_evaluator
    test/test_evaluator.cpp
  )
  target_link_libraries(test_evaluator
    ${PROJECT_NAME}
    autoware_trajectory_ranker_plugins
  )

  ament_add_gtest(test_metrics
    test/test_metrics.cpp
  )
  target_link_libraries(test_metrics
    ${PROJECT_NAME}
    autoware_trajectory_ranker_plugins
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
