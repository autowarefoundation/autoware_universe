file(GLOB_RECURSE CUDA_TARGET_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cu)

find_package(OpenCV 4.5.4 REQUIRED)

foreach (T_FILE IN LISTS CUDA_TARGET_SRCS)
  get_filename_component(T_NAME ${T_FILE} NAME_WE)
  add_executable(${T_NAME} ${T_FILE})
  target_include_directories(${T_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
    ${CMAKE_SOURCE_DIR}/lib/MPPI-Generic/include/mppi/dynamics/racer_dubins
    ${CMAKE_SOURCE_DIR}/lib/MPPI-Generic/include/mppi/dynamics/dubins
    ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(${T_NAME}
    mppi_path_tracking
    mppi_viz_gl
    ${OpenCV_LIBS})
endforeach()
