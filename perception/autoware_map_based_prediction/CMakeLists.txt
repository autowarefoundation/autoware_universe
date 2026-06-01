cmake_minimum_required(VERSION 3.14)
project(autoware_map_based_prediction)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_auto_add_library(map_based_prediction_node SHARED
  src/map_based_prediction_node.cpp
  lib/map_based_prediction_node/callbacks.cpp
  lib/map_based_prediction_node/diagnostics.cpp
  lib/path_generator/path_generator.cpp
  lib/path_generator/frenet.cpp
  lib/predictor_vehicle/predictor_vehicle.cpp
  lib/predictor_vehicle/object_processing.cpp
  lib/predictor_vehicle/maneuver_prediction.cpp
  lib/predictor_vehicle/path_processing.cpp
  lib/predictor_vehicle/debug.cpp
  lib/predictor_vru/predictor_vru.cpp
  lib/predictor_vru/history.cpp
  lib/predictor_vru/traffic_signal.cpp
  lib/predictor_vru/fence.cpp
  lib/utils.cpp
)

target_link_libraries(map_based_prediction_node
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

autoware_agnocast_wrapper_register_node(map_based_prediction_node
  PLUGIN "autoware::map_based_prediction::MapBasedPredictionNode"
  EXECUTABLE map_based_prediction
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
)

## Tests
if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE test_files test/**/*.cpp)
  ament_add_ros_isolated_gtest(test_map_based_prediction ${test_files})

  target_link_libraries(test_map_based_prediction
  map_based_prediction_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
