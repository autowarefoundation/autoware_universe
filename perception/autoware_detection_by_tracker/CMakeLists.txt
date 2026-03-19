cmake_minimum_required(VERSION 3.14)
project(autoware_detection_by_tracker)

find_package(autoware_cmake REQUIRED)
find_package(autoware_agnocast_wrapper REQUIRED)
autoware_package()

# Ignore -Wnonportable-include-path in Clang for mussp
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

### Find PCL Dependencies
find_package(PCL REQUIRED)

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/detection_by_tracker_node.cpp
  src/tracker/tracker_handler.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${autoware_agnocast_wrapper_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  autoware_agnocast_wrapper
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

autoware_agnocast_wrapper_setup(${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::detection_by_tracker::DetectionByTracker"
  EXECUTABLE detection_by_tracker_node
  AGNOCAST_EXECUTOR SingleThreadedAgnocastExecutor
)
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
