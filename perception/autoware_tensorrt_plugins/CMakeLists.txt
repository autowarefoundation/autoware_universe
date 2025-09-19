cmake_minimum_required(VERSION 3.14)
project(autoware_tensorrt_plugins)

find_package(autoware_cmake REQUIRED)
autoware_package()

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
  # Note: cublas_device was depreciated in CUDA version 9.2
  #       https://forums.developer.nvidia.com/t/where-can-i-find-libcublas-device-so-or-libcublas-device-a/67251/4
  #       In LibTorch, CUDA_cublas_device_LIBRARY is used.
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
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
# try to find the CUDNN module
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

# set flags for spconv availability
option(SPCONV_AVAIL "spconv available" OFF)
# try to find spconv
find_package(cumm)
find_package(spconv)
if(${cumm_FOUND} AND ${spconv_FOUND})
  message(STATUS "spconv is available!")
  set(SPCONV_AVAIL ON)
else()
  message(WARNING "spconv is NOT available")
  set(SPCONV_AVAIL OFF)
endif()

if(NOT TRT_AVAIL OR CUDA_AVAIL OR CUDNN_AVAIL OR SPCONV_AVAIL)
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
endif()

find_package(tensorrt_cmake_module REQUIRED)
find_package(TENSORRT)
if(TENSORRT_VERSION VERSION_LESS 10.0)
  message(WARNING "Unsupported version TensorRT ${TENSORRT_VERSION} detected. This package requires TensorRT 10.0 or later.")
  return()
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
endif()

add_definitions("-DTV_CUDA")

# cSpell:ignore expt gencode
list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr -diag-suppress 1675 --extended-lambda")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_75,code=sm_75")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
# NOTE(knzo25): PTX support for newer GPUs until we can compile directly
list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")
# TODO(knzo25): enable when the driver supports it
#list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=sm_120")

cuda_add_library(cuda_ops SHARED
  src/argsort_ops/argsort.cu
  src/bev_ops/bev_pool_cuda.cu
  src/scatter_ops/segment_csr.cu
  src/unique_ops/unique.cu
  src/multi_scale_deform_attn_ops/ms_deform_attn_kernel.cu
  src/rotate_ops/rotate_kernel.cu
  src/select_and_pad_ops/select_and_pad_kernel.cu
)

add_library(${PROJECT_NAME} SHARED
  src/argsort_plugin.cpp
  src/argsort_plugin_creator.cpp
  src/quick_cumsum_cuda_plugin.cpp
  src/quick_cumsum_cuda_plugin_creator.cpp
  src/get_indices_pairs_implicit_gemm_plugin_creator.cpp
  src/get_indices_pairs_implicit_gemm_plugin.cpp
  src/get_indices_pairs_plugin_creator.cpp
  src/get_indices_pairs_plugin.cpp
  src/implicit_gemm_plugin_creator.cpp
  src/implicit_gemm_plugin.cpp
  src/indice_conv_plugin_creator.cpp # cSpell:ignore indice
  src/indice_conv_plugin.cpp
  src/multi_scale_deformable_attention_plugin.cpp
  src/multi_scale_deformable_attention_plugin_creator.cpp
  src/rotate_plugin.cpp
  src/rotate_plugin_creator.cpp
  src/segment_csr_plugin_creator.cpp
  src/segment_csr_plugin.cpp
  src/select_and_pad_plugin.cpp
  src/select_and_pad_plugin_creator.cpp
  src/unique_plugin.cpp
  src/unique_plugin_creator.cpp
  src/plugin_registration.cpp
  src/plugin_utils.cpp
)


target_compile_definitions(${PROJECT_NAME} PRIVATE _GLIBCXX_USE_CXX11_ABI=1)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${NVINFER}
    CUDA::cudart
    cuda_ops
    spconv::spconv
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/plugins
)

ament_auto_package()
