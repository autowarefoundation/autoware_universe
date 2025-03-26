cmake_minimum_required(VERSION 3.14)
project(autoware_cuda_pointcloud_preprocessor)

find_package(ament_cmake_auto REQUIRED)
find_package(CUDA)

if(NOT ${CUDA_FOUND})
  message(WARNING "cuda was not found, so the autoware_cuda_pointcloud_preprocessor package will not be built.")
  return()
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-g -G")
endif()

ament_auto_find_build_dependencies()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

include_directories(
    include
    SYSTEM
    ${CUDA_INCLUDE_DIRS}
)

list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -diag-suppress 20012") # cSpell: ignore expt

cuda_add_library(cuda_pointcloud_preprocessor_lib SHARED
  src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  src/cuda_pointcloud_preprocessor/common_kernels.cu
  src/cuda_pointcloud_preprocessor/organize_kernels.cu
  src/cuda_pointcloud_preprocessor/outlier_kernels.cu
  src/cuda_pointcloud_preprocessor/undistort_kernels.cu
)

target_link_libraries(cuda_pointcloud_preprocessor_lib
  ${autoware_pointcloud_preprocessor_TARGETS}
)

target_include_directories(cuda_pointcloud_preprocessor_lib SYSTEM PRIVATE
  ${autoware_pointcloud_preprocessor_INCLUDE_DIRS}
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
)


# Targets
ament_auto_add_library(cuda_pointcloud_preprocessor SHARED
  src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor_node.cpp
)

target_link_libraries(cuda_pointcloud_preprocessor
  ${CUDA_LIBRARIES}
  cuda_pointcloud_preprocessor_lib
)

rclcpp_components_register_node(cuda_pointcloud_preprocessor
  PLUGIN "autoware::cuda_pointcloud_preprocessor::CudaPointcloudPreprocessorNode"
  EXECUTABLE cuda_pointcloud_preprocessor_node
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
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
