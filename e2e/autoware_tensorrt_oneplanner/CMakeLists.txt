cmake_minimum_required(VERSION 3.14)
project(autoware_tensorrt_oneplanner)

find_package(autoware_cmake REQUIRED)
autoware_package()

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
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message(WARNING "CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
find_library(NVINFER nvinfer)
find_library(NVONNXPARSER nvonnxparser)
if(NVINFER AND NVONNXPARSER)
  set(TRT_AVAIL ON)
else()
  message(WARNING "TensorRT is NOT available")
  set(TRT_AVAIL OFF)
endif()

# spconv is required by the BEVFusion preprocessing/plugin stack
option(SPCONV_AVAIL "spconv available" OFF)
find_package(cumm)
find_package(spconv)
if(${cumm_FOUND} AND ${spconv_FOUND})
  set(SPCONV_AVAIL ON)
else()
  message(WARNING "spconv is NOT available")
  set(SPCONV_AVAIL OFF)
endif()

# The BEVFusion CUDA library (voxelization kernels) and the diffusion-planner
# component library (lane preprocessing, postprocessing, turn-indicator manager)
# are installed but not exported as linkable targets by ament, so locate them
# explicitly.
find_library(BEVFUSION_CUDA_LIB autoware_bevfusion_cuda_lib)
find_library(DIFFUSION_PLANNER_LIB autoware_diffusion_planner_component)

if(TRT_AVAIL AND CUDA_AVAIL AND SPCONV_AVAIL AND BEVFUSION_CUDA_LIB AND DIFFUSION_PLANNER_LIB)
  add_compile_options(-Wno-narrowing)

  ament_auto_add_library(${PROJECT_NAME}_component SHARED
    src/bev_encoder.cpp
    src/planner_inference.cpp
    src/oneplanner_core.cpp
    src/oneplanner_node.cpp
  )

  target_include_directories(${PROJECT_NAME}_component PUBLIC
    ${CUDA_INCLUDE_DIRS}
    ${autoware_cuda_utils_INCLUDE_DIRS}
  )

  target_link_libraries(${PROJECT_NAME}_component
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${BEVFUSION_CUDA_LIB}
    ${DIFFUSION_PLANNER_LIB}
  )

  rclcpp_components_register_node(${PROJECT_NAME}_component
    PLUGIN "autoware::tensorrt_oneplanner::OnePlannerNode"
    EXECUTABLE ${PROJECT_NAME}_node
  )

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
      config
  )
else()
  message(WARNING
    "Skipping ${PROJECT_NAME} build (TRT_AVAIL=${TRT_AVAIL}, CUDA_AVAIL=${CUDA_AVAIL}, "
    "SPCONV_AVAIL=${SPCONV_AVAIL}, BEVFUSION_CUDA_LIB=${BEVFUSION_CUDA_LIB}, DIFFUSION_PLANNER_LIB=${DIFFUSION_PLANNER_LIB})")
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
  )
endif()
