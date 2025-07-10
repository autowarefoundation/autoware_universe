cmake_minimum_required(VERSION 3.22)
project(autoware_diffusion_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Options
option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

# Find CUDA
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message(STATUS "CUDA is available!")
    message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
    message(STATUS "CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message(WARNING "CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# Find TensorRT libraries
option(TRT_AVAIL "TensorRT available" OFF)
find_library(NVINFER nvinfer)
find_library(NVONNXPARSER nvonnxparser)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message(STATUS "TensorRT is available!")
    message(STATUS "NVINFER: ${NVINFER}")
    message(STATUS "NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message(WARNING "TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# Find CUDNN library (optional, often used with TRT)
option(CUDNN_AVAIL "CUDNN available" OFF)
find_library(CUDNN_LIBRARY
  NAMES libcudnn.so
  PATHS $ENV{LD_LIBRARY_PATH} ${CUDA_TOOLKIT_ROOT_DIR}/lib64
  DOC "CUDNN library."
)
if(CUDNN_LIBRARY)
  if(CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif()
  set(CUDNN_AVAIL ON)
else()
  message(WARNING "CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# Add compile option to suppress deprecation warnings (optional)
add_compile_options(-Wno-deprecated-declarations)

# Suppress narrowing conversion warnings from TensorRT headers
add_compile_options(-Wno-narrowing)

# Add your component library
ament_auto_add_library(autoware_diffusion_planner_component SHARED
  src/diffusion_planner_node.cpp
  src/conversion/lanelet.cpp
  src/conversion/agent.cpp
  src/conversion/ego.cpp
  src/postprocessing/postprocessing_utils.cpp
  src/preprocessing/lane_segments.cpp
  src/preprocessing/preprocessing_utils.cpp
  src/preprocessing/traffic_signals.cpp
  src/utils/utils.cpp
  src/utils/marker_utils.cpp
)


if(TRT_AVAIL AND CUDA_AVAIL)
  target_include_directories(autoware_diffusion_planner_component PUBLIC
    ${CUDA_INCLUDE_DIRS}
  )
  target_link_libraries(autoware_diffusion_planner_component
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
  )
endif()

if(CUDNN_AVAIL)
  target_link_libraries(autoware_diffusion_planner_component
    ${CUDNN_LIBRARY}
  )
endif()

rclcpp_components_register_node(autoware_diffusion_planner_component
  PLUGIN "autoware::diffusion_planner::DiffusionPlanner"
  EXECUTABLE autoware_diffusion_planner_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_diffusion_planner
    tests/agent_test.cpp
    tests/lanelet_test.cpp
    tests/ego_test.cpp
    tests/lane_segments_test.cpp
    tests/pre_processing_utils_test.cpp
    tests/fixed_queue_test.cpp
    tests/postprocessing_utils_test.cpp
    tests/utils_test.cpp
    tests/marker_utils_test.cpp
    tests/lanelet_integration_test.cpp
    tests/postprocessing_utils_edge_case_test.cpp
    tests/preprocessing_utils_edge_case_test.cpp
    tests/agent_edge_case_test.cpp
    tests/lanelet_edge_case_test.cpp
    tests/diffusion_planner_integration_test.cpp)

  target_link_libraries(test_diffusion_planner autoware_diffusion_planner_component)
  ament_target_dependencies(test_diffusion_planner
    autoware_test_utils
  )

  endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  test_map
)
