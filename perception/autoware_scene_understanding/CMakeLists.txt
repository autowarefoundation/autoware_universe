cmake_minimum_required(VERSION 3.14)
project(scene_understanding_tensorrt)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find CUDA
find_package(CUDA REQUIRED)

# Find TensorRT
set(TensorRT_ROOT "/usr/src/tensorrt" CACHE PATH "TensorRT root directory")
find_path(TensorRT_INCLUDE_DIR NvInfer.h
  HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES include)
find_library(TensorRT_LIBRARY nvinfer
  HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)
find_library(TensorRT_ONNX_PARSER_LIBRARY nvonnxparser
  HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)

if(NOT TensorRT_INCLUDE_DIR OR NOT TensorRT_LIBRARY OR NOT TensorRT_ONNX_PARSER_LIBRARY)
  message(FATAL_ERROR "TensorRT not found. Please set TensorRT_ROOT.")
endif()

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SceneUnderstanding.msg"
  DEPENDENCIES std_msgs
)

# Create TensorRT library
add_library(${PROJECT_NAME}_lib SHARED
  src/tensorrt_engine.cpp
  src/scene_understanding_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TensorRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib
  ${TensorRT_LIBRARY}
  ${TensorRT_ONNX_PARSER_LIBRARY}
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
)

# Create visualization library
add_library(${PROJECT_NAME}_visualization_lib SHARED
  src/scene_visualization_node.cpp
)

target_include_directories(${PROJECT_NAME}_visualization_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_visualization_lib
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}_visualization_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_filters
)

# Make libraries depend on generated messages
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")
target_link_libraries(${PROJECT_NAME}_visualization_lib "${cpp_typesupport_target}")

# Register components
rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "scene_understanding_tensorrt::SceneUnderstandingNode"
  EXECUTABLE scene_understanding_node_component
)

rclcpp_components_register_node(${PROJECT_NAME}_visualization_lib
  PLUGIN "scene_understanding_tensorrt::SceneVisualizationNode"
  EXECUTABLE scene_visualization_node_component
)

# Create executables
add_executable(scene_understanding_node
  src/scene_understanding_node_main.cpp
)

target_link_libraries(scene_understanding_node
  ${PROJECT_NAME}_lib
)

add_executable(scene_visualization_node
  src/scene_visualization_node_main.cpp
)

target_link_libraries(scene_visualization_node
  ${PROJECT_NAME}_visualization_lib
)

# Install
install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_visualization_lib
  scene_understanding_node
  scene_visualization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_visualization_lib)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_filters
)

ament_package()
