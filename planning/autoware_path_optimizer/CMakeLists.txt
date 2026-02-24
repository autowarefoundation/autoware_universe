cmake_minimum_required(VERSION 3.14)
project(autoware_path_optimizer)

include(CheckCXXCompilerFlag)

# Run MPT generator script at configure time is removed because it depends on acados_template
# which is only available at build time when building acados from source.
# Generation is now handled by add_custom_command in src/acados_mpc/CMakeLists.txt.

# NOTE: With the following option, when the compile-time and runtime CPU
#       architectures are different, the node will die.
# For Eigen vectorization.
# check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
# if(COMPILER_SUPPORTS_MARCH_NATIVE)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
#   message(STATUS "Enabling MARCH NATIVE ")
# endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(autoware_path_optimizer SHARED
  # node
  src/node.cpp
  # core algorithms
  src/replan_checker.cpp
  src/mpt_optimizer.cpp
  src/state_equation_generator.cpp
  # debug marker
  src/debug_marker.cpp
  # vehicle model
  src/vehicle_model/vehicle_model_interface.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  # utils
  src/utils/trajectory_utils.cpp
  src/utils/geometry_utils.cpp
)

target_include_directories(autoware_path_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Paths to universe external (acados; provisioned by ansible-playbook autoware.dev_env.setup_acados)
get_filename_component(UNIVERSE_EXTERNAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../external" ABSOLUTE)
set(ACADOS_WORKSPACE_DIR "${UNIVERSE_EXTERNAL_DIR}/acados" CACHE INTERNAL "")
set(TERA_RENDERER_SOURCE_DIR "${UNIVERSE_EXTERNAL_DIR}/tera_renderer" CACHE INTERNAL "")

# Add acados_mpc subdir first to set up acados paths
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc)

target_include_directories(autoware_path_optimizer
  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/acados>
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/acados_c>
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/blasfeo/include>
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/hpipm/include>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/acados>
    $<INSTALL_INTERFACE:include/acados_c>
    $<INSTALL_INTERFACE:include/blasfeo/include>
    $<INSTALL_INTERFACE:include/hpipm/include>
)

# Link against acados helper library
target_link_libraries(autoware_path_optimizer acados_interface)

# Ensure package library depends on acados_interface and the generator
add_dependencies(autoware_path_optimizer acados_interface generate_mpt ${ACADOS_TARGET})

# register node
rclcpp_components_register_node(autoware_path_optimizer
  PLUGIN "autoware::path_optimizer::PathOptimizer"
  EXECUTABLE path_optimizer_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_path_optimizer_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

# Export acados libraries for downstream packages
# We also need to export the include directories for the acados headers
# Export base include directory and subdirectories needed for blasfeo/hpipm
ament_export_include_directories(include)
ament_export_include_directories(include/blasfeo/include)
ament_export_include_directories(include/hpipm/include)
ament_export_include_directories(include/acados)
ament_export_include_directories(include/acados_c)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    rviz
)

install(PROGRAMS
  scripts/calculation_time_plotter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install acados_interface library and headers
install(TARGETS acados_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES src/acados_mpc/include/acados_interface.hpp
  DESTINATION include
)

install(DIRECTORY src/acados_mpc/c_generated_code/
  DESTINATION include/c_generated_code
  FILES_MATCHING PATTERN "*.h"
)
