cmake_minimum_required(VERSION 3.22)
project(autoware_cuda_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(CUDA)

if(NOT ${CUDA_FOUND})
  message(WARNING "cuda is not found, so the autoware_cuda_utils package won't be built.")
  return()
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmake/find_cccl.cmake)

# The installed headers use Thrust and cuda_runtime_api.h, so consumers need both include paths.
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE CCCL::CCCL autoware_cuda_utils::cccl_system CUDA::cudart)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()

  cuda_add_executable(test_autoware_cuda_utils
    test/test_cuda_check_error.cu
    test/test_cuda_unique_ptr.cu
    test/test_cuda_utils.cu
    test/test_stream_unique_ptr.cu
    test/test_thrust_utils.cu
    test/test_main.cpp
  )

  target_link_libraries(test_autoware_cuda_utils
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
  )

  target_include_directories(test_autoware_cuda_utils
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    SYSTEM
    ${GTEST_INCLUDE_DIRS}
  )

  ament_add_gtest_test(test_autoware_cuda_utils)
endif()

ament_auto_package(CONFIG_EXTRAS cmake/find_cccl.cmake)
