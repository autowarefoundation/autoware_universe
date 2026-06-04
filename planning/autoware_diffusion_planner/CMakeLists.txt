cmake_minimum_required(VERSION 3.14)
project(autoware_diffusion_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(Boost REQUIRED)
find_package(onnxruntime QUIET)

# Options
option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)
option(BUILD_TOOL "Build tools for autoware_diffusion_planner" OFF)

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

if(TRT_AVAIL AND CUDA_AVAIL)
  if(NOT onnxruntime_FOUND)
    find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
      HINTS /usr/local/include /usr/local/include/onnxruntime
    )
    find_library(ONNXRUNTIME_LIBRARY onnxruntime
      HINTS /usr/local/lib /usr/local/lib64
    )
    if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
      set(onnxruntime_FOUND TRUE)
    endif()
  endif()

  if(onnxruntime_FOUND)
    message(STATUS "ONNX Runtime is available; enabling ONNX Runtime inference support")
  else()
    message(WARNING "ONNX Runtime is NOT Available; ONNX Runtime inference support will be disabled")
  endif()

  # Add compile option to suppress deprecation warnings (optional)
  add_compile_options(-Wno-deprecated-declarations)

  # Suppress narrowing conversion warnings from TensorRT headers
  add_compile_options(-Wno-narrowing)

  set(DIFFUSION_PLANNER_SOURCES
    src/diffusion_planner_core.cpp
    src/diffusion_planner_node.cpp
    src/conversion/lanelet.cpp
    src/conversion/agent.cpp
    src/postprocessing/postprocessing_utils.cpp
    src/preprocessing/lane_segments.cpp
    src/preprocessing/preprocessing_utils.cpp
    src/preprocessing/traffic_signals.cpp
    src/utils/utils.cpp
    src/utils/marker_utils.cpp
    src/utils/planning_factor_utils.cpp
    src/postprocessing/turn_indicator_manager.cpp
    src/inference/solver/dpm_solver.cpp
    src/inference/guidance/guidance.cpp
    src/inference/guidance/centerline_guidance.cpp
    src/inference/guidance/start_guidance.cpp
    src/inference/guidance/stop_guidance.cpp
    src/inference/guidance/utils.cpp
    src/inference/multi_step_inference.cpp
    src/inference/single_step_inference.cpp
    src/inference/utils.cpp
  )
  if(onnxruntime_FOUND)
    list(APPEND DIFFUSION_PLANNER_SOURCES
      src/inference/onnxruntime_inference.cpp
    )
  endif()

  # Add your component library
  ament_auto_add_library(autoware_diffusion_planner_component SHARED
    ${DIFFUSION_PLANNER_SOURCES}
  )

  target_include_directories(autoware_diffusion_planner_component PUBLIC
    ${CUDA_INCLUDE_DIRS}
    ${autoware_cuda_utils_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )
  if(onnxruntime_FOUND)
    target_include_directories(autoware_diffusion_planner_component PUBLIC
      ${ONNXRUNTIME_INCLUDE_DIR}
    )
    target_compile_definitions(autoware_diffusion_planner_component PUBLIC
      AUTOWARE_DIFFUSION_PLANNER_USE_ONNXRUNTIME
    )
  endif()
  target_link_libraries(autoware_diffusion_planner_component
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
  )

  if(onnxruntime_FOUND)
    if(TARGET onnxruntime::onnxruntime)
      target_link_libraries(autoware_diffusion_planner_component onnxruntime::onnxruntime)
    elseif(TARGET onnxruntime)
      target_link_libraries(autoware_diffusion_planner_component onnxruntime)
    else()
      target_link_libraries(autoware_diffusion_planner_component ${ONNXRUNTIME_LIBRARY})
    endif()
  endif()

  rclcpp_components_register_node(autoware_diffusion_planner_component
    PLUGIN "autoware::diffusion_planner::DiffusionPlanner"
    EXECUTABLE autoware_diffusion_planner_node
  )

  if(BUILD_TESTING)
    ament_add_ros_isolated_gtest(test_diffusion_planner
      test/agent_test.cpp
      test/lanelet_test.cpp
      test/lane_segments_test.cpp
      test/pre_processing_utils_test.cpp
      test/postprocessing_utils_test.cpp
      test/utils_test.cpp
      test/marker_utils_test.cpp
      test/lanelet_integration_test.cpp
      test/postprocessing_utils_edge_case_test.cpp
      test/preprocessing_utils_edge_case_test.cpp
      test/agent_edge_case_test.cpp
      test/lanelet_edge_case_test.cpp
      test/diffusion_planner_integration_test.cpp
      test/rtc_prefix_test.cpp
      test/planning_factor_utils_test.cpp)

    target_link_libraries(test_diffusion_planner autoware_diffusion_planner_component)
    ament_target_dependencies(test_diffusion_planner
      ament_index_cpp
      autoware_lanelet2_extension
      autoware_map_msgs
      autoware_perception_msgs
      autoware_test_utils
    )
  endif()

  if(BUILD_TOOL)
    if(onnxruntime_FOUND)
      include(FetchContent)
      find_package(ZLIB REQUIRED)
      fetchcontent_declare(
        CLI11
        GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
        GIT_TAG v2.4.2
      )
      fetchcontent_makeavailable(CLI11)

      add_library(cnpy STATIC
        tool/cnpy/cnpy.cpp
      )
      target_include_directories(cnpy PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/tool/cnpy
      )
      target_link_libraries(cnpy PUBLIC
        ZLIB::ZLIB
      )

      add_executable(validate_inference_mode_consistency
        tool/validate_inference_mode_consistency.cpp
      )
      target_include_directories(validate_inference_mode_consistency PUBLIC
        ${CUDA_INCLUDE_DIRS}
        ${autoware_cuda_utils_INCLUDE_DIRS}
      )
      target_link_libraries(validate_inference_mode_consistency
        autoware_diffusion_planner_component
        ${NVINFER}
        ${NVONNXPARSER}
        ${CUDA_LIBRARIES}
        ${CUBLAS_LIBRARIES}
        ${CUDNN_LIBRARY}
        CLI11::CLI11
        cnpy
      )
      ament_target_dependencies(validate_inference_mode_consistency
        ament_index_cpp
      )
      install(TARGETS validate_inference_mode_consistency
        DESTINATION lib/${PROJECT_NAME}
      )
    else()
      message(WARNING "Skipping validate_inference_mode_consistency because ONNX Runtime is unavailable")
    endif()
  endif()
endif()
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  test_map
)
