cmake_minimum_required(VERSION 3.14)
project(autoware_tensorrt_bevformer LANGUAGES CXX CUDA)

if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)
endif()

# Set C++ and CUDA standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

# Set CUDA architecture for Turing GPUs (sm_75)
set(ARCH "75")

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_internal_perception_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# TensorRT setup - adjust path to your installation
set(TENSORRT_ROOT "/usr/local/TensorRT-10.8.0.43")
include_directories(${TENSORRT_ROOT}/include)
link_directories(${TENSORRT_ROOT}/lib)

# Add NvOnnxParser dependency
find_library(NVONNXPARSER_LIB nvonnxparser PATHS ${TENSORRT_PATH}/lib)

find_package(ament_index_cpp REQUIRED)

# Include TensorRT plugin common directory
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/TensorRT/common
)

# Set TensorRT libraries with the correct variable name
set(TENSORRT_LIBS nvinfer nvinfer_plugin nvonnxparser)

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

# Completely override CUDA flags to avoid autoware's problematic settings
set(CMAKE_CUDA_FLAGS "")
set(CMAKE_CUDA_FLAGS_DEBUG "")
set(CMAKE_CUDA_FLAGS_RELEASE "")
set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "")
set(CMAKE_CUDA_FLAGS_MINSIZEREL "")

# Set CUDA architecture through CUDA_ARCHITECTURES property
set_property(TARGET ${TENSORRT_OPS_TARGET} PROPERTY CUDA_ARCHITECTURES ${ARCH})

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

target_link_libraries(${TENSORRT_OPS_TARGET} PUBLIC cudnn ${TENSORRT_LIBS} cublas)

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

ament_target_dependencies(autoware_tensorrt_bevformer
  rclcpp
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
)
# Link main library with the TensorRT plugins and other dependencies
target_link_libraries(${PROJECT_NAME}
  ${CUDA_LIBRARIES}
  cudart
  ${OpenCV_LIBRARIES}
  ${TENSORRT_LIBS}
  ${NVONNXPARSER_LIB}
  ${TENSORRT_OPS_TARGET}  # Link with our custom TensorRT plugins
  dl  # Add dl library for dlopen/dlsym
  ament_index_cpp::ament_index_cpp
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
