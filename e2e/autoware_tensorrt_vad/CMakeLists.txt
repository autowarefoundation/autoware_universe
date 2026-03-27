cmake_minimum_required(VERSION 3.14)

project(autoware_tensorrt_vad)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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

if(TRT_AVAIL AND CUDA_AVAIL)
  # Find additional dependencies
  find_package(eigen3_cmake_module REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(OpenCV REQUIRED)

  include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CUDA_INCLUDE_DIRS}
  )

  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
  endif()

  # CUDA architecture settings
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
  if(CUDA_VERSION VERSION_GREATER_EQUAL "12.8")
    if(CUDA_VERSION VERSION_LESS "13.0")
      list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_101,code=sm_101")
    else()  # CUDA 13.0 renamed SM101 to SM110
      list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_110,code=sm_110")
    endif()
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=sm_120")
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=compute_120")
  else()
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=compute_89")
  endif()

  cuda_add_library(${PROJECT_NAME}_cuda_lib SHARED
    lib/networks/preprocess/multi_camera_preprocess_kernel.cu
    lib/networks/postprocess/map_postprocess_kernel.cu
    lib/networks/postprocess/object_postprocess_kernel.cu
  )

  target_link_libraries(${PROJECT_NAME}_cuda_lib
    ${OpenCV_LIBS}
  )

  ament_auto_add_library(${PROJECT_NAME}_lib SHARED
    lib/data_types.cpp
    lib/vad_interface.cpp
    lib/vad_config.cpp
    lib/synchronization_strategy.cpp
    lib/coordinate_transformer.cpp
    lib/converter.cpp
    lib/input_converter/image_converter.cpp
    lib/input_converter/transform_matrix_converter.cpp
    lib/input_converter/can_bus_converter.cpp
    lib/input_converter/bev_shift_converter.cpp
    lib/output_converter/trajectory_converter.cpp
    lib/output_converter/map_converter.cpp
    lib/output_converter/objects_converter.cpp
    lib/networks/net.cpp
    lib/networks/backbone.cpp
    lib/networks/head.cpp
    lib/networks/tensor.cpp
    lib/networks/preprocess/multi_camera_preprocess.cpp
    lib/networks/postprocess/map_postprocess.cpp
    lib/networks/postprocess/object_postprocess.cpp
  )

  target_link_libraries(${PROJECT_NAME}_lib
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_cuda_lib
  )

  target_include_directories(${PROJECT_NAME}_lib
    SYSTEM PUBLIC
    ${CUDA_INCLUDE_DIRS}
  )

  ament_auto_add_library(${PROJECT_NAME}_component SHARED
    src/vad_node.cpp
  )

  target_link_libraries(${PROJECT_NAME}_component
    ${PROJECT_NAME}_lib
  )

  rclcpp_components_register_node(${PROJECT_NAME}_component
    PLUGIN "autoware::tensorrt_vad::VadNode"
    EXECUTABLE vad_node
  )

  install(
    TARGETS ${PROJECT_NAME}_cuda_lib
    DESTINATION lib
  )

endif()

ament_auto_package(
  INSTALL_TO_SHARE
      launch
      config
)
