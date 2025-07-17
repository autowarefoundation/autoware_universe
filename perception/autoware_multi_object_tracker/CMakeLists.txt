cmake_minimum_required(VERSION 3.14)
project(autoware_multi_object_tracker)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Ignore -Wnonportable-include-path in Clang for mussp
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

# Check if AVX2 is supported
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)

if(COMPILER_SUPPORTS_AVX2)
# cspell:ignore mfma
    add_compile_options(-mavx2 -mfma -O3)  # Enable AVX2 & FMA
endif()

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

# Generate exe file
set(${PROJECT_NAME}_src
  src/multi_object_tracker_node.cpp
  src/debugger/debugger.cpp
  src/debugger/debug_object.cpp
  src/processor/processor.cpp
  src/processor/input_manager.cpp
)
set(${PROJECT_NAME}_lib
  lib/odometry.cpp
  lib/association/association.cpp
  lib/association/mu_successive_shortest_path/mu_ssp.cpp
  lib/object_model/types.cpp
  lib/object_model/shapes.cpp
  lib/tracker/motion_model/bicycle_motion_model.cpp
  # cspell: ignore ctrv
  lib/tracker/motion_model/ctrv_motion_model.cpp
  lib/tracker/motion_model/cv_motion_model.cpp
  lib/tracker/motion_model/static_motion_model.cpp
  lib/tracker/model/tracker_base.cpp
  lib/tracker/model/vehicle_tracker.cpp
  lib/tracker/model/multiple_vehicle_tracker.cpp
  lib/tracker/model/pedestrian_tracker.cpp
  lib/tracker/model/pedestrian_and_bicycle_tracker.cpp
  lib/tracker/model/unknown_tracker.cpp
  lib/tracker/model/pass_through_tracker.cpp
  lib/uncertainty/uncertainty_processor.cpp
  lib/tracker/util/adaptive_threshold_cache.cpp
)
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_src}
  ${${PROJECT_NAME}_lib}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  glog::glog
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::multi_object_tracker::MultiObjectTracker"
  EXECUTABLE multi_object_tracker_node
)

## Tests
if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(autoware_perception_msgs REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE test_files test/test_multi_object_tracker.cpp
    test/test_bench.cpp
    test/test_utils.cpp
  )
  add_definitions(-D_SRC_RESOURCES_DIR_PATH="${PROJECT_SOURCE_DIR}/test/data/")
  ament_add_ros_isolated_gtest(test_multi_object_tracker ${test_files})

  target_link_libraries(test_multi_object_tracker
  ${PROJECT_NAME}
  ${autoware_perception_msgs_LIBRARIES}
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
