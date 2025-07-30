cmake_minimum_required(VERSION 3.14)
project(autoware_tensorrt_bevformer LANGUAGES CXX)

if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Check for CUDA availability first
find_package(CUDA QUIET)
find_package(CUDAToolkit QUIET)

# Check for TensorRT and CUDNN
if(CUDAToolkit_FOUND)
  find_package(cudnn_cmake_module QUIET)
  find_package(tensorrt_cmake_module QUIET)
  if(cudnn_cmake_module_FOUND)
    find_package(CUDNN QUIET)
  endif()
  if(tensorrt_cmake_module_FOUND)
    find_package(TENSORRT QUIET)
  endif()
endif()

# If CUDA dependencies are not found, skip building this package
if(NOT (CUDAToolkit_FOUND AND CUDNN_FOUND AND TENSORRT_FOUND))
  message(WARNING "CUDA, CUDNN, or TensorRT libraries are not found. Skipping ${PROJECT_NAME} build.")
  ament_auto_package()
  return()
endif()

# Enable CUDA language only after confirming CUDA is available
enable_language(CUDA)

# Set CUDA standards
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

# Set CUDA architecture for Turing GPUs (sm_75)
set(ARCH "75")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_internal_perception_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${CUDAToolkit_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/TensorRT/common
)

# Build TensorRT plugins library first
# Get all TensorRT plugin source files
file(GLOB_RECURSE TENSORRT_SRCS
  TensorRT/common/*.cpp
  TensorRT/common/*.cu
  TensorRT/plugin/*/*.cpp
  TensorRT/plugin/*/*.cu
)

# Print what TensorRT files are being compiled (for debugging)
message(STATUS "TensorRT plugin source files to be compiled:")
foreach(src ${TENSORRT_SRCS})
  message(STATUS "  ${src}")
endforeach()

# Create TensorRT plugins library
set(TENSORRT_OPS_TARGET tensorrt_ops)
add_library(${TENSORRT_OPS_TARGET} SHARED ${TENSORRT_SRCS})

# Set CUDA architecture through CUDA_ARCHITECTURES property
set_property(TARGET ${TENSORRT_OPS_TARGET} PROPERTY CUDA_ARCHITECTURES ${ARCH})

# Completely override CUDA flags to avoid autoware's problematic settings
set(CMAKE_CUDA_FLAGS "")
set(CMAKE_CUDA_FLAGS_DEBUG "")
set(CMAKE_CUDA_FLAGS_RELEASE "")
set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "")
set(CMAKE_CUDA_FLAGS_MINSIZEREL "")

# Set target properties to handle CUDA compilation properly
set_target_properties(${TENSORRT_OPS_TARGET} PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_RESOLVE_DEVICE_SYMBOLS ON
)

# Override compile options completely for this target
set_target_properties(${TENSORRT_OPS_TARGET} PROPERTIES
  COMPILE_OPTIONS ""
  COMPILE_FLAGS ""
)

# Add minimal compile options
target_compile_options(${TENSORRT_OPS_TARGET} PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:
    -fPIC
    -O2
  >
  $<$<COMPILE_LANGUAGE:CUDA>:
    --compiler-options=-fPIC,-O2
  >
)

target_link_libraries(${TENSORRT_OPS_TARGET} PUBLIC 
  ${CUDNN_LIBRARY} 
  ${TENSORRT_LIBRARIES}
  CUDA::cublas
  CUDA::cudart
)

# Main project source files
set(SOURCES
  src/bevformer_node.cpp
  src/bevformer_preprocessor.cpp
  src/bevformer_data_loader.cpp
  src/bevformer_data_manager.cpp
  src/bevformer_inference_engine.cpp
  src/ros_utils.cpp
  src/marker_utils.cpp
  src/postprocessing/postprocessing.cpp
  src/preprocessing/preprocessing_pipeline.cpp
  src/preprocessing/normalize_multiview_image.cpp
  src/preprocessing/multi_scale_flip_aug_3d.cpp
  src/preprocessing/compose.cpp
  src/preprocessing/augmentation_transforms.cpp
)

# Main library
ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
  ament_index_cpp
)

# Link main library with the TensorRT plugins and other dependencies
target_link_libraries(${PROJECT_NAME}
  CUDA::cudart
  ${OpenCV_LIBRARIES}
  ${TENSORRT_LIBRARIES}
  ${TENSORRT_OPS_TARGET}
  dl
)

# Add dependency to ensure TensorRT plugins are built first
add_dependencies(${PROJECT_NAME} ${TENSORRT_OPS_TARGET})

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -Wpedantic
  -Wno-deprecated-declarations
  -Wno-unused-parameter
  -Wno-unused-function
  -Wno-unused-variable
  -Wno-unused-but-set-variable
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_export_libraries(${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::tensorrt_bevformer::TRTBEVFormerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(DIRECTORY
  include
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install TensorRT plugins library
install(TARGETS ${TENSORRT_OPS_TARGET}
  LIBRARY DESTINATION lib
)

ament_auto_package()