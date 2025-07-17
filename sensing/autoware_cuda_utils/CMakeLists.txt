cmake_minimum_required(VERSION 3.22)
project(autoware_cuda_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(CUDA)

if(NOT ${CUDA_FOUND})
  message(WARNING "cuda is not found, so the autoware_cuda_utils package won't be built.")
  return()
endif()

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

ament_auto_package()
