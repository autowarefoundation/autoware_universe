cmake_minimum_required(VERSION 3.8)
project(autoware_ground_segmentation_cuda)

find_package(ament_cmake_auto REQUIRED)
find_package(CUDA)
find_package(agnocastlib)
find_package(autoware_vehicle_info_utils REQUIRED)

if(NOT ${CUDA_FOUND})
  message(WARNING "cuda was not found, so the autoware_ground_segmentation_cuda package will not be built.")
  return()
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-g -G")
endif()

if(USE_AGNOCAST AND NOT agnocastlib_FOUND)
  message(FATAL_ERROR "agnocastlib is required when USE_AGNOCAST is enabled")
endif()

ament_auto_find_build_dependencies()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

if(USE_AGNOCAST)
    add_definitions(-DUSE_AGNOCAST_ENABLED)
endif()

if(BUILD_TESTING)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

include_directories(
    include
    SYSTEM
    ${CUDA_INCLUDE_DIRS}
)

# cSpell: ignore expt gencode
list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -diag-suppress 20012")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_75,code=sm_75")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")

#################### cuda_ground_segmentation ##################
cuda_add_library(cuda_ground_segmentation_lib SHARED
  src/cuda_scan_ground_segmentation/cuda_scan_ground_segmentation_filter.cu
)

target_link_libraries(cuda_ground_segmentation_lib
  ${autoware_pointcloud_preprocessor_TARGETS}
  ${autoware_cuda_pointcloud_preprocessor_TARGETS}
)

target_include_directories(cuda_ground_segmentation_lib SYSTEM PRIVATE
  ${autoware_pointcloud_preprocessor_INCLUDE_DIRS}
  ${autoware_cuda_pointcloud_preprocessor_INCLUDE_DIRS}
  ${autoware_point_types_INCLUDE_DIRS}
  ${cuda_blackboard_INCLUDE_DIRS}
  ${diagnostic_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${autoware_cuda_utils_INCLUDE_DIRS}
  ${autoware_vehicle_info_utils_INCLUDE_DIRS}
)

if(USE_AGNOCAST)
    target_include_directories(cuda_ground_segmentation_lib SYSTEM PRIVATE
        ${autoware_agnocast_wrapper_INCLUDE_DIRS}
        ${agnocastlib_INCLUDE_DIRS}
    )
    target_link_libraries(cuda_ground_segmentation_lib
        ${agnocastlib_LIBRARIES}
    )
endif()


# Targets
ament_auto_add_library(cuda_ground_segmentation SHARED
  src/cuda_scan_ground_segmentation/cuda_scan_ground_segmentation_filter_node.cpp
)

target_link_libraries(cuda_ground_segmentation
  ${CUDA_LIBRARIES}
  ${diagnostic_msgs_LIBRARIES}
  cuda_ground_segmentation_lib
)

#=========== ScanGround Segmentation filter ========
rclcpp_components_register_node(cuda_ground_segmentation
  PLUGIN "autoware::cuda_ground_segmentation::CudaScanGroundSegmentationFilterNode"
  EXECUTABLE cuda_scan_ground_segmentation_filter_node)

################################################################################
# Install
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS cuda_ground_segmentation_lib
  LIBRARY DESTINATION lib
)

ament_auto_package()

# Set ROS_DISTRO macros
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
    add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "foxy")
    add_compile_definitions(ROS_DISTRO_FOXY)
elseif(${ROS_DISTRO} STREQUAL "galactic")
    add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
    add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()
