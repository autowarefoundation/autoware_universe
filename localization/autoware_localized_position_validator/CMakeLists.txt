cmake_minimum_required(VERSION 3.14)
project(autoware_localized_position_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()

# By default, this package will be skipped in the build process.
# use:
#   --cmake-args -DSKIP_THIS_PACKAGE=OFF --packages-up-to autoware_localized_position_validator
# or just rewrite this flag to OFF for building
option(SKIP_THIS_PACKAGE "Skip building this package" ON)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)
option(CUDA_AVAIL "CUDA available" OFF)
option(TRT_AVAIL "TensorRT available" OFF)
option(CUDNN_AVAIL "CUDNN available" OFF)
option(Torch_AVAIL "libtorch available" OFF)

if (SKIP_THIS_PACKAGE)
  message("Skipping this package: SKIP_THIS_PACKAGE==ON")
else ()
  # setup LibTorch
  message(WARNING "checking LibTorch...")
  execute_process(
    # download official build, might not suit your CUDA version
    #COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/download_libtorch.sh
    # clone and compile the LibTorch
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/clone_libtorch.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # Set LibTorch paths
  set(Torch_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch/share/cmake/Torch")
  set(Caffe2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch/share/cmake/Caffe2")
  #set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch/")

  find_package(Torch REQUIRED)
  if(Torch_FOUND)
    message("LibTorch is available!")
    set(Torch_AVAIL ON)
  else()
    message("LibTorch NOT FOUND")
  endif()

  # CUDA
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
  endif()

  # TensorRT
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
    message("TensorRT is NOT Available")
  endif()

  # cuDNN
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
    message("CUDNN is NOT Available")
  endif()

  if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL AND Torch_AVAIL)
    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    include_directories(
      lib
      ${CUDA_INCLUDE_DIRS}
      ${TORCH_INCLUDE_DIRS}
    )

    ament_auto_add_library(${PROJECT_NAME}_lib SHARED
      lib/preprocess/voxelization.cpp
      lib/preprocess/voxelization_cpu.cpp
      lib/position_validator.cpp
    )

    cuda_add_library(${PROJECT_NAME}_cuda_lib SHARED
      lib/preprocess/voxelization.cu
    )

    target_link_libraries(${PROJECT_NAME}_lib
      ${NVINFER}
      ${NVONNXPARSER}
      ${CUDA_LIBRARIES}
      ${CUBLAS_LIBRARIES}
      ${CUDNN_LIBRARY}
      ${PROJECT_NAME}_cuda_lib
      ${TORCH_LIBRARIES}
    )

    target_include_directories(${PROJECT_NAME}_lib
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${TORCH_INCLUDE_DIRS}
    )

    # To suppress unknown-pragmas error. The root-cause is CUB library in CUDA 11.6.
    # This issue was fixed by https://github.com/NVIDIA/cub/commit/7d608bf1dc14553e2fb219eabeed80b76621b6fe
    target_include_directories(${PROJECT_NAME}_lib
      SYSTEM PUBLIC
      ${CUDA_INCLUDE_DIRS}
    )

    ament_auto_add_library(${PROJECT_NAME}_component SHARED
      src/node.cpp
    )

    target_link_libraries(${PROJECT_NAME}_component
      ${PROJECT_NAME}_lib
    )

    rclcpp_components_register_node(${PROJECT_NAME}_component
      PLUGIN "autoware::localized_position_validator::LocalizedPositionValidatorNode"
      EXECUTABLE ${PROJECT_NAME}_node
    )

    ament_auto_package(
      INSTALL_TO_SHARE
        launch
        config
    )

    install(
      TARGETS ${PROJECT_NAME}_cuda_lib
      DESTINATION lib
    )

  else()
    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    ament_auto_package(
      INSTALL_TO_SHARE
        launch
    )
  endif()
endif()
