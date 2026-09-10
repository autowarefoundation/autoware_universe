cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_ranker)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_trajectory_ranker plugins.xml)

generate_parameter_library(${PROJECT_NAME}_param
  param/trajectory_ranker_parameter_struct.yaml
)

include_directories(
  SYSTEM
)

set(trajectory_ranker_lib_target ${PROJECT_NAME}_lib)

ament_auto_add_library(${trajectory_ranker_lib_target} SHARED
  src/trajectory_ranker_wrapper.cpp
  src/trajectory_ranker.cpp
  src/evaluation.cpp
  src/utils.cpp
)

target_link_libraries(${trajectory_ranker_lib_target}
  ${PROJECT_NAME}_param
)

target_compile_options(${trajectory_ranker_lib_target} PUBLIC
  -Wno-error=deprecated-declarations
)

ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  src/metrics/lateral_acceleration_metric.cpp
  src/metrics/longitudinal_jerk_metric.cpp
  src/metrics/distance_metric.cpp
  src/metrics/lateral_deviation_metric.cpp
  src/metrics/steering_consistency_metric.cpp
  src/metrics/trajectory_consistency_metric.cpp
  src/metrics/metrics_utils.cpp
)

target_link_libraries(${PROJECT_NAME}_plugins
  ${trajectory_ranker_lib_target}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_utils
    test/test_utils.cpp
  )
  target_link_libraries(test_utils
  ${trajectory_ranker_lib_target}
  )

  ament_add_gtest(test_metrics_utils
    test/test_metrics_utils.cpp
  )
  target_link_libraries(test_metrics_utils
    ${PROJECT_NAME}_plugins
  )

  ament_add_gtest(test_evaluator
    test/test_evaluator.cpp
  )
  target_link_libraries(test_evaluator
    ${trajectory_ranker_lib_target}
    ${PROJECT_NAME}_plugins
  )

  ament_add_gtest(test_metrics
    test/test_metrics.cpp
  )
  target_link_libraries(test_metrics
    ${trajectory_ranker_lib_target}
    ${PROJECT_NAME}_plugins
  )

  ament_add_gtest(test_trajectory_ranker_integration
    test/test_trajectory_ranker_integration.cpp
  )
  target_link_libraries(test_trajectory_ranker_integration
    ${trajectory_ranker_lib_target}
    ${PROJECT_NAME}_plugins
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
)
