cmake_minimum_required(VERSION 3.14)
project(autoware_tensorrt_bevformer)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wno-deprecated-declarations)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
find_library(NVINFER nvinfer)
find_library(NVONNXPARSER nvonnxparser)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
find_library(CUDNN_LIBRARY
NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
PATH_SUFFIXES lib lib64 bin
DOC "CUDNN library."
)
if(CUDNN_LIBRARY)
  if(CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif()
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT available")
  set(CUDNN_AVAIL OFF)
endif()

# Only build if ALL dependencies are available
if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  # NOW we can safely enable CUDA language since we know CUDA is available
  enable_language(CUDA)

  # Set CUDA standards
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

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
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/TensorRT/common
  )

  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
  endif()

  # Set CUDA architecture for Turing GPUs (sm_75) and newer
  list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -diag-suppress 1675 --extended-lambda")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_75,code=sm_75")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
  # PTX support for newer GPUs
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")

  # Build TensorRT plugins library first
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

  # Create TensorRT plugins library using cuda_add_library
  set(TENSORRT_OPS_TARGET tensorrt_ops)
  cuda_add_library(${TENSORRT_OPS_TARGET} SHARED ${TENSORRT_SRCS})

  target_compile_options(${TENSORRT_OPS_TARGET} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:
      -fPIC
      -O2
      -Wno-deprecated-declarations
      -Wno-unused-parameter
      -Wno-unused-function
      -Wno-unused-variable
      -Wno-unused-but-set-variable
      -Wno-comment
      -Wno-maybe-uninitialized
      -Wno-switch
    >
  )

  target_link_libraries(${TENSORRT_OPS_TARGET}
    ${CUDNN_LIBRARY}
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
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
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDNN_LIBRARY}
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

  # To suppress unknown-pragmas error
  target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
    ${CUDA_INCLUDE_DIRS}
  )

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

else()
  # If dependencies are not available, create a minimal package
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  message(WARNING "Required dependencies (CUDA, TensorRT, CUDNN) not found. Skipping ${PROJECT_NAME} build.")

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
  )
endif()
