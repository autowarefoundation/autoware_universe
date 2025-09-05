cmake_minimum_required(VERSION 3.14)
project(autoware_imu_corrector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(imu_corrector SHARED
  src/imu_corrector_core.cpp
)

ament_auto_add_library(gyro_bias_estimator SHARED
  src/gyro_bias_estimator.cpp
  src/gyro_bias_estimation_module.cpp
)

target_include_directories(gyro_bias_estimator
  PUBLIC include
)

rclcpp_components_register_node(imu_corrector
  PLUGIN "autoware::imu_corrector::ImuCorrector"
  EXECUTABLE imu_corrector_node
  EXECUTOR SingleThreadedExecutor
)

rclcpp_components_register_node(gyro_bias_estimator
  PLUGIN "autoware::imu_corrector::GyroBiasEstimator"
  EXECUTABLE gyro_bias_estimator_node
  EXECUTOR SingleThreadedExecutor
)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})
  ament_add_gmock(${test_name} ${filepath})
  target_link_libraries("${test_name}" gyro_bias_estimator)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_testcase(test/test_gyro_bias_estimation_module.cpp)
  ament_add_ros_isolated_gtest(test_gyro_bias_scale_estimation_module
    test/test_main.cpp
    test/test_gyro_bias_scale_estimation_module.cpp
  )
  target_link_libraries(test_gyro_bias_scale_estimation_module gyro_bias_estimator)
  ament_target_dependencies(test_gyro_bias_scale_estimation_module
    ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
  )
  target_include_directories(test_gyro_bias_scale_estimation_module
    PUBLIC include
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
