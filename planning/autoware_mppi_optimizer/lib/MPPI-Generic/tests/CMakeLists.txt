add_executable(collision_check_gpu_test collision_check_gpu_test.cu)
target_include_directories(collision_check_gpu_test PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
target_link_libraries(collision_check_gpu_test
  mppi_path_tracking
  mppi_viz_gl)
