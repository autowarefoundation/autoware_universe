cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_classifier)

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
    message(STATUS "CUDA is available!")
    message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
    message(STATUS "CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  set(CUDA_AVAIL ON)
else()
  message(STATUS "CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVONNXPARSER nvonnxparser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVONNXPARSER AND NVINFER_PLUGIN)
  if(CUDA_VERBOSE)
    message(STATUS "TensorRT is available!")
    message(STATUS "NVINFER: ${NVINFER}")
    message(STATUS "NVINFER_PLUGIN: ${NVINFER_PLUGIN}")
    message(STATUS "NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message(STATUS "TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

if(TRT_AVAIL AND CUDA_AVAIL)
  add_definitions(-DENABLE_GPU)

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )

  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/classifier/color_classifier.cpp
    src/classifier/cnn_classifier.cpp
    src/classifier/cnn_lamp_recognizer.cpp
    src/traffic_light_classifier.cpp
    src/traffic_light_classifier_node.cpp
    src/traffic_light_classifier_process.cpp
  )
  target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    stdc++fs
  )
  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "autoware::traffic_light::TrafficLightClassifierNodelet"
    EXECUTABLE traffic_light_classifier_node
  )

  ament_auto_add_library(single_image_debug_inference_node SHARED
    src/classifier/cnn_classifier.cpp
    src/classifier/color_classifier.cpp
    src/classifier/cnn_lamp_recognizer.cpp
    src/traffic_light_classifier.cpp
    src/traffic_light_classifier_node.cpp
    src/traffic_light_classifier_process.cpp
    src/single_image_debug_inference_node.cpp
  )
  target_link_libraries(single_image_debug_inference_node
    ${OpenCV_LIBRARIES}
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    stdc++fs
    opencv_core
    opencv_highgui
  )
  rclcpp_components_register_node(single_image_debug_inference_node
    PLUGIN "autoware::traffic_light::SingleImageDebugInferenceNode"
    EXECUTABLE single_image_debug_inference
  )

else()
  message(STATUS "CUDA and/or TensorRT were not found. build only color classifier")

  include_directories(
    ${OpenCV_INCLUDE_DIRS}
  )

  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/classifier/color_classifier.cpp
    src/traffic_light_classifier.cpp
    src/traffic_light_classifier_node.cpp
    src/traffic_light_classifier_process.cpp
  )
  target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
  )

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "autoware::traffic_light::TrafficLightClassifierNodelet"
    EXECUTABLE traffic_light_classifier_node
  )

endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_utils
    test/test_utils.cpp
  )

  # ROS-free unit tests for the classification core.
  ament_auto_add_gtest(test_traffic_light_classifier
    test/test_traffic_light_classifier.cpp
  )
  target_link_libraries(test_traffic_light_classifier
    ${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
  )

  # ROS-free unit tests for the color classification core (ColorClassifierCore).
  ament_auto_add_gtest(test_color_classifier
    test/test_color_classifier.cpp
  )
  target_link_libraries(test_color_classifier
    ${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
  )

  # ColorClassifier ROS adapter tests (the size guard needs the node's logger).
  ament_add_ros_isolated_gtest(test_color_classifier_adapter
    test/test_color_classifier_adapter.cpp
  )
  target_link_libraries(test_color_classifier_adapter
    ${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
  )

  # Node-level integration tests (topic-driven pub/sub + diagnostics).
  ament_add_ros_isolated_gtest(test_traffic_light_classifier_integration
    test/test_traffic_light_classifier_integration.cpp
  )
  target_link_libraries(test_traffic_light_classifier_integration
    ${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
  )

  # Tests that depend on TensorRT/CUDA code, built only when those libraries are available.
  if(TRT_AVAIL AND CUDA_AVAIL)
    # CNNClassifierCore tests (ROS-free): static helpers run GPU-free; the classify()
    # tests self-skip without a GPU/model. TIMEOUT covers the engine build.
    ament_auto_add_gtest(test_cnn_classifier
      test/test_cnn_classifier.cpp
      TIMEOUT 600
    )
    target_link_libraries(test_cnn_classifier
      ${PROJECT_NAME}
      ${OpenCV_LIBRARIES}
      ${CUDA_LIBRARIES}
    )
  endif()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  test/test_data
)
