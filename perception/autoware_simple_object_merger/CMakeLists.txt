cmake_minimum_required(VERSION 3.5)
project(autoware_simple_object_merger)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
)

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/simple_object_merger_base.cpp
  src/simple_detected_object_merger_node.cpp
  src/simple_tracked_object_merger_node.cpp
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::simple_object_merger::SimpleDetectedObjectMergerNode"
  EXECUTABLE simple_object_merger_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::simple_object_merger::SimpleTrackedObjectMergerNode"
  EXECUTABLE simple_tracked_object_merger_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

# Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  if(NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    ament_auto_add_gtest(simple_object_merger_node_tests
      test/test_node.cpp
    )
  endif()
endif()

# Package
autoware_ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
