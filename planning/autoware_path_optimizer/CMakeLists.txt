cmake_minimum_required(VERSION 3.14)
project(autoware_path_optimizer)

include(CheckCXXCompilerFlag)

# Run MPT generator script at configure time so generated code exists
execute_process(
  COMMAND ${CMAKE_COMMAND} -E echo "Running generate_mpt.sh"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc
)
execute_process(
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc/generators/path_tracking_mpc_spatial_with_body_points.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc
  RESULT_VARIABLE _generate_mpt_result
  OUTPUT_VARIABLE _generate_mpt_out
  ERROR_VARIABLE _generate_mpt_err
)
if(NOT _generate_mpt_result EQUAL 0)
  message(FATAL_ERROR "generate_mpt.sh failed: ${_generate_mpt_err}")
endif()

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

target_include_directories(autoware_path_optimizer
  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    /home/arjunram/Workspace/acados/include
    /home/arjunram/Workspace/acados/include/acados
    /home/arjunram/Workspace/acados/include/acados_c
    /home/arjunram/Workspace/acados/include/blasfeo/include
    /home/arjunram/Workspace/acados/include/hpipm/include
)

# Link against acados helper library which depends on generated code
target_link_libraries(autoware_path_optimizer acados_interface acados_generated_code)

# register node
rclcpp_components_register_node(autoware_path_optimizer
  PLUGIN "autoware::path_optimizer::PathOptimizer"
  EXECUTABLE path_optimizer_node
)

# Add acados_mpc subdir which contains generator script and generated-code target
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc)

# Ensure package library depends on generated acados code and generator target
add_dependencies(autoware_path_optimizer acados_generated_code generate_mpt)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_path_optimizer_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

# Export acados libraries for downstream packages
ament_export_targets(export_acados_libs HAS_LIBRARY_TARGET)

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

# Install acados_interface library and headers for downstream packages
install(TARGETS acados_interface acados_generated_code
  EXPORT export_acados_libs
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
