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
file(GLOB_RECURSE SIMPL_SOURCES src/*.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED ${SIMPL_SOURCES})

rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN "autoware::simpl::SimplNode" EXECUTABLE
  ${PROJECT_NAME}_exe)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # autoware::simpl::archetype
  ament_auto_add_gtest(test_agent test/archetype/test_agent.cpp)
  target_link_libraries(test_agent ${PROJECT_NAME})

  ament_auto_add_gtest(test_fixed_queue test/archetype/test_fixed_queue.cpp)
  target_link_libraries(test_fixed_queue ${PROJECT_NAME})

  ament_auto_add_gtest(test_map test/archetype/test_map.cpp)
  target_link_libraries(test_map ${PROJECT_NAME})

  ament_auto_add_gtest(test_prediction test/archetype/test_prediction.cpp)
  target_link_libraries(test_prediction ${PROJECT_NAME})

  # autoware::simpl::conversion
  ament_auto_add_gtest(test_predicted_object
                       test/conversion/test_predicted_object.cpp)
  target_link_libraries(test_predicted_object ${PROJECT_NAME})

  ament_auto_add_gtest(test_tracked_object
                       test/conversion/test_tracked_object.cpp)
  target_link_libraries(test_tracked_object ${PROJECT_NAME})

  # autoware::simpl::processing
  ament_auto_add_gtest(test_geometry test/processing/test_geometry.cpp)
  target_link_libraries(test_geometry ${PROJECT_NAME})

  ament_auto_add_gtest(test_polyline test/processing/test_polyline.cpp)
  target_link_libraries(test_polyline ${PROJECT_NAME})

  ament_auto_add_gtest(test_postprocessor
                       test/processing/test_postprocessor.cpp)
  target_link_libraries(test_postprocessor ${PROJECT_NAME})

  ament_auto_add_gtest(test_preprocessor test/processing/test_preprocessor.cpp)
  target_link_libraries(test_preprocessor ${PROJECT_NAME})

  ament_auto_add_gtest(test_rpe test/processing/test_rpe.cpp)
  target_link_libraries(test_rpe ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE launch config data)
