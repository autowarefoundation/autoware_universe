cmake_minimum_required(VERSION 3.14)
project(autoware_calibration_status_classifier)

find_package(autoware_cmake REQUIRED)
autoware_package()

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
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL)
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  include_directories(
    include
    ${CUDA_INCLUDE_DIRS}
  )

  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
    set(CUDA_NVCC_FLAGS "-g -G")
  endif()

  ament_auto_add_library(${PROJECT_NAME}_lib SHARED
    lib/calibration_status_classifier.cpp
  )

  # cSpell:ignore expt
  list(APPEND CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_86,code=sm_86")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_87,code=sm_87")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_89,code=sm_89")
  if(CUDA_VERSION VERSION_LESS "13.0")
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_101,code=sm_101")
  else()  # CUDA 13.0 renamed SM101 to SM110
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_110,code=sm_110")
  endif()
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=sm_120")
  list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_120,code=compute_120")

  cuda_add_library(${PROJECT_NAME}_cuda_lib SHARED
    lib/preprocess/preprocess_cuda.cu
  )

  target_include_directories(${PROJECT_NAME}_cuda_lib SYSTEM PRIVATE
    ${autoware_cuda_utils_INCLUDE_DIRS}
  )

  target_link_libraries(${PROJECT_NAME}_lib
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${PROJECT_NAME}_cuda_lib
  )

  target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  # To suppress unknown-pragmas error. The root-cause is CUB library in CUDA 11.6.
  # This issue was fixed by https://github.com/NVIDIA/cub/commit/7d608bf1dc14553e2fb219eabeed80b76621b6fe
  target_include_directories(${PROJECT_NAME}_lib
    SYSTEM PUBLIC
    ${CUDA_INCLUDE_DIRS}
  )

  ament_auto_add_library(${PROJECT_NAME}_component SHARED
    src/calibration_status_classifier_node.cpp
  )

  target_link_libraries(${PROJECT_NAME}_component
    ${PROJECT_NAME}_lib
  )

  rclcpp_components_register_node(${PROJECT_NAME}_component
    PLUGIN "autoware::calibration_status_classifier::CalibrationStatusClassifierNode"
    EXECUTABLE calibration_status_classifier_node
  )

  install(
    TARGETS ${PROJECT_NAME}_cuda_lib
    DESTINATION lib
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_ros REQUIRED)
    ament_auto_find_test_dependencies()

    find_package(OpenCV REQUIRED)
    find_package(ZLIB REQUIRED)

    add_executable(test_preprocessing
      test/test_preprocessing.cpp
    )
    target_include_directories(test_preprocessing PUBLIC
      ${autoware_point_types_INCLUDE_DIRS}
    )
    target_link_libraries(test_preprocessing
      ${PROJECT_NAME}_lib
      gtest
      gtest_main
      ament_index_cpp::ament_index_cpp
    )
    ament_target_dependencies(test_preprocessing
      OpenCV
      ZLIB
    )
    ament_add_test(test_preprocessing
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "$<TARGET_FILE:test_preprocessing>"
    )

    add_executable(test_model_inference
      test/test_model_inference.cpp
    )
    target_include_directories(test_model_inference PUBLIC
      ${autoware_point_types_INCLUDE_DIRS}
    )
    target_link_libraries(test_model_inference
      ${PROJECT_NAME}_lib
      gtest
      gtest_main
      ament_index_cpp::ament_index_cpp
    )
    ament_target_dependencies(test_model_inference
      OpenCV
      ZLIB
    )
    ament_add_test(test_model_inference
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "$<TARGET_FILE:test_model_inference>"
    )

    add_executable(test_calibration_status_classifier
      test/test_calibration_status_classifier.cpp
    )
    target_include_directories(test_calibration_status_classifier PUBLIC
      ${autoware_point_types_INCLUDE_DIRS}
    )
    target_link_libraries(test_calibration_status_classifier
      ${PROJECT_NAME}_lib
      gtest
      gtest_main
      ament_index_cpp::ament_index_cpp
    )
    ament_target_dependencies(test_calibration_status_classifier
      OpenCV
      ZLIB
    )
    ament_add_test(test_calibration_status_classifier
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "$<TARGET_FILE:test_calibration_status_classifier>"
    )

    # Suppress pedantic warnings for test target (PCL)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(test_preprocessing PRIVATE -Wno-pedantic)
      target_compile_options(test_model_inference PRIVATE -Wno-pedantic)
      target_compile_options(test_calibration_status_classifier PRIVATE -Wno-pedantic)
    endif()
  endif()

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
      config
      test/data
  )

else()
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
      config
  )
endif()
