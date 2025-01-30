cmake_minimum_required(VERSION 3.14)
project(autoware_simpl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- find dependencies ---
find_package(autoware_cmake REQUIRED)
autoware_package()

# --- find CUDA/CUDNN/TensorRT dependencies ---
find_package(CUDA REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(CUDNN REQUIRED)
find_package(TENSORRT REQUIRED)

# --- check CUDA tools and TensorRT version ---
if(NOT
   (CUDAToolkit_FOUND
    AND CUDNN_FOUND
    AND TENSORRT_FOUND))
  message(FATAL_ERROR "cuda, cudnn, tensorrt libraries are not found")
endif()

if(TENSORRT_VERSION VERSION_LESS 8.5)
  message(
    FATAL_ERROR
      "Unsupported version TensorRT ${TENSORRT_VERSION} detected. This package requires TensorRT 8.5 or later."
  )
endif()

# --- link targets ---
ament_auto_add_library(${PROJECT_NAME} SHARED src/simpl_node.cpp)

rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN "autoware::simpl::SimplNode" EXECUTABLE
  ${PROJECT_NAME}_exe)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
