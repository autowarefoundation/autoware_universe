cmake_minimum_required(VERSION 3.14)
project(autoware_object_sorter)

# Dependencies
find_package(autoware_cmake REQUIRED)
find_package(autoware_agnocast_wrapper REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/object_sorter_base.cpp
  src/detected_object_sorter_node.cpp
  src/tracked_object_sorter_node.cpp
)

# Agnocast Method 2: the sorter nodes inherit autoware::agnocast_wrapper::Node (via the shared
# ObjectSorterBase template). Pull in the wrapper (and, transitively, agnocastlib) include dirs
# and define USE_AGNOCAST_ENABLED when Agnocast is enabled.
target_include_directories(${PROJECT_NAME} PRIVATE
  ${autoware_agnocast_wrapper_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  autoware_agnocast_wrapper
)

autoware_agnocast_wrapper_setup(${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::object_sorter::DetectedObjectSorterNode"
  EXECUTABLE detected_object_sorter_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::object_sorter::TrackedObjectSorterNode"
  EXECUTABLE tracked_object_sorter_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

# Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
