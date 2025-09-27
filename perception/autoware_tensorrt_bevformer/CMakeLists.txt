# cspell:ignore expt, gencode, multiview
cmake_minimum_required(VERSION 3.14)
project(autoware_tensorrt_bevformer)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)
endif()

add_compile_options(-Wno-deprecated-declarations)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

# CUDA availability check
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

# Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# TensorRT
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

# CUDNN
option(CUDNN_AVAIL "CUDNN available" OFF)
find_library(CUDNN_LIBRARY
  NAMES libcudnn.so libcudnn
  PATHS $ENV{LD_LIBRARY_PATH} ${CUDNN_ROOT_DIR} ${CMAKE_INSTALL_PREFIX}
  PATH_SUFFIXES lib lib64 bin
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

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  # CUDA flags
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --expt-relaxed-constexpr -diag-suppress 1675 --extended-lambda")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_75,code=sm_75")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")

  # Build TensorRT plugin library
  file(GLOB_RECURSE TENSORRT_SRCS
    TensorRT/common/*.cpp
    TensorRT/common/*.cu
    TensorRT/plugin/*/*.cpp
    TensorRT/plugin/*/*.cu
  )
  message(STATUS "TensorRT plugin source files:")
  foreach(src ${TENSORRT_SRCS})
    message(STATUS "  ${src}")
  endforeach()

  set(TENSORRT_OPS_TARGET tensorrt_ops)
  cuda_add_library(${TENSORRT_OPS_TARGET} SHARED ${TENSORRT_SRCS})
  target_compile_options(${TENSORRT_OPS_TARGET} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:
      -fPIC -O2 -Wno-deprecated-declarations -Wno-unused-parameter -Wno-unused-function
      -Wno-unused-variable -Wno-unused-but-set-variable -Wno-comment -Wno-switch>
  )
  target_link_libraries(${TENSORRT_OPS_TARGET}
    ${CUDNN_LIBRARY}
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
  )
  target_include_directories(${TENSORRT_OPS_TARGET}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/TensorRT/common
  )
  # Separate SYSTEM include to avoid warnings in TensorRT headers with clang-tidy
  target_include_directories(${TENSORRT_OPS_TARGET}
    SYSTEM PRIVATE
      ${CUDA_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
  )

  # Main source files
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

  ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES})
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    visualization_msgs
    tf2_geometry_msgs
    ament_index_cpp
    Eigen3
  )

  target_link_libraries(${PROJECT_NAME}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDNN_LIBRARY}
    ${TENSORRT_OPS_TARGET}
    dl
  )

  # Enables us to catch missing symbols at link time
  target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)

  add_dependencies(${PROJECT_NAME} ${TENSORRT_OPS_TARGET})

  target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall -Wextra -Wpedantic -Wno-deprecated-declarations -Wno-unused-parameter
    -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable
  )

  target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

  target_include_directories(${PROJECT_NAME}
    PUBLIC
    # Project's public headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    PRIVATE
    # Internal-only headers
    ${CMAKE_CURRENT_SOURCE_DIR}/TensorRT/common
    SYSTEM PUBLIC
    # System/third-party includes (propagate to consumers when PUBLIC)
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "autoware::tensorrt_bevformer::TRTBEVFormerNode"
    EXECUTABLE ${PROJECT_NAME}_node
  )

  install(DIRECTORY src launch config DESTINATION share/${PROJECT_NAME})
  install(TARGETS ${TENSORRT_OPS_TARGET} LIBRARY DESTINATION lib)

  ament_auto_package()
else()
  # Dependencies missing
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
  message(WARNING "Required dependencies (CUDA, TensorRT, CUDNN) not found. Skipping ${PROJECT_NAME} build.")
  ament_auto_package(INSTALL_TO_SHARE launch)
endif()
