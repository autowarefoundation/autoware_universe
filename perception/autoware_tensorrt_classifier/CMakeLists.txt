cmake_minimum_required(VERSION 3.5)
project(autoware_tensorrt_classifier)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -Wno-write-strings -fopenmp -Wall")

find_package(autoware_cmake REQUIRED)
autoware_package()

# TODO(amadeuszsz): Remove -Wno-deprecated-declarations once removing implicit quantization
add_compile_options(-Wno-deprecated-declarations)

find_package(CUDA)
find_package(CUDNN)
find_package(TENSORRT)
find_package(OpenCV REQUIRED)

if(NOT (${CUDA_FOUND} AND ${CUDNN_FOUND} AND ${TENSORRT_FOUND}))
  message(WARNING "cuda, cudnn, tensorrt libraries are not found")
  return()
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tensorrt_classifier.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  OpenCV
)

cuda_add_library(${PROJECT_NAME}_gpu_preprocess
  SHARED
  src/preprocess.cu
)

target_include_directories(${PROJECT_NAME}_gpu_preprocess
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${TENSORRT_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${TENSORRT_LIBRARIES}
  ${PROJECT_NAME}_gpu_preprocess
  stdc++fs
)

install(
  TARGETS ${PROJECT_NAME}_gpu_preprocess
  LIBRARY DESTINATION lib
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  TENSORRT_VERSION_MAJOR=${TENSORRT_VERSION_MAJOR}
)

list(APPEND ${PROJECT_NAME}_LIBRARIES "${PROJECT_NAME}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(CUDA)
ament_export_dependencies(cudnn_cmake_module)
ament_export_dependencies(CUDNN)
ament_export_dependencies(tensorrt_cmake_module)
ament_export_dependencies(TENSORRT)

ament_auto_package()
