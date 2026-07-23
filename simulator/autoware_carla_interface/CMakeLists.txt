cmake_minimum_required(VERSION 3.8)
project(autoware_carla_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake_python for Python support
find_package(ament_cmake_python REQUIRED)

# ============================================================================
# C++ Components - CARLA Utility Nodes
# ============================================================================

# CarlaStatePublisher: Synchronizes pose/twist into odometry
ament_auto_add_library(${PROJECT_NAME}_state_component SHARED
  src/carla_state_publisher.cpp
)

target_compile_features(${PROJECT_NAME}_state_component PUBLIC cxx_std_17)

rclcpp_components_register_node(${PROJECT_NAME}_state_component
  PLUGIN "autoware::carla_interface::CarlaStatePublisher"
  EXECUTABLE carla_state_publisher
)

# CarlaOperationModePublisher: Publishes operation mode state
ament_auto_add_library(${PROJECT_NAME}_operation_mode_component SHARED
  src/carla_operation_mode_publisher.cpp
)

target_compile_features(${PROJECT_NAME}_operation_mode_component PUBLIC cxx_std_17)

rclcpp_components_register_node(${PROJECT_NAME}_operation_mode_component
  PLUGIN "autoware::carla_interface::CarlaOperationModePublisher"
  EXECUTABLE carla_operation_mode_publisher
)

# ============================================================================
# SplatSim gRPC proto (fetched from the tier4/splatsim release, generated here)
# ============================================================================
# The generated *_pb2*.py modules are not committed; they are regenerated on
# every build from the pinned SplatSim release proto so they stay in sync with
# the installed protobuf/grpc runtime.
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(SPLATSIM_VERSION "v1.2.0")
set(SPLATSIM_PROTO_URL
  "https://raw.githubusercontent.com/tier4/splatsim/${SPLATSIM_VERSION}/proto/splatsim/v1/rendering_service.proto")
set(_splatsim_proto_dir "${CMAKE_CURRENT_BINARY_DIR}/splatsim_proto")
set(_splatsim_proto_file "${_splatsim_proto_dir}/rendering_service.proto")
set(_splatsim_proto_out "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/splatsim/proto")

file(MAKE_DIRECTORY "${_splatsim_proto_dir}")
message(STATUS "Fetching SplatSim proto ${SPLATSIM_VERSION} from ${SPLATSIM_PROTO_URL}")
file(DOWNLOAD "${SPLATSIM_PROTO_URL}" "${_splatsim_proto_file}"
  STATUS _splatsim_dl_status
  TLS_VERIFY ON)
list(GET _splatsim_dl_status 0 _splatsim_dl_code)
if(NOT _splatsim_dl_code EQUAL 0)
  list(GET _splatsim_dl_status 1 _splatsim_dl_msg)
  message(FATAL_ERROR "Failed to download SplatSim proto (${SPLATSIM_VERSION}): ${_splatsim_dl_msg}")
endif()

execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/generate_splatsim_proto.py"
    --proto "${_splatsim_proto_file}"
    --out-dir "${_splatsim_proto_out}"
  RESULT_VARIABLE _splatsim_gen_result)
if(NOT _splatsim_gen_result EQUAL 0)
  message(FATAL_ERROR "Failed to generate SplatSim proto Python modules")
endif()

# ============================================================================
# Python Package Installation
# ============================================================================

# Install Python package
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Install Python entry point scripts
install(PROGRAMS
  scripts/autoware_carla_interface
  scripts/multi_camera_combiner
  scripts/spectator_follow
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# Install Directories
# ============================================================================

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# Install calibration maps
install(DIRECTORY calibration_maps/ DESTINATION share/${PROJECT_NAME}/)

# Install resource files
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME}/)

# Skip flake8 test to avoid conflict with black formatter in pre-commit
if(BUILD_TESTING)
  set(ament_cmake_flake8_FOUND FALSE)
endif()

ament_auto_package()
