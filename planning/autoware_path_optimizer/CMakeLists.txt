cmake_minimum_required(VERSION 3.14)
project(autoware_path_optimizer)

include(CheckCXXCompilerFlag)

# NOTE: With the following option, when the compile-time and runtime CPU
#       architectures are different, the node will die.
# For Eigen vectorization.
# check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
# if(COMPILER_SUPPORTS_MARCH_NATIVE)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
#   message(STATUS "Enabling MARCH NATIVE ")
# endif()

find_package(autoware_cmake REQUIRED)

# Exclude generated acados code from copyright checking
# Must be set before autoware_package() is called (when lint hooks are registered)
# Use glob patterns to exclude all generated files (they're created during build)
if(NOT DEFINED AMENT_LINT_AUTO_FILE_EXCLUDE)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE "" CACHE INTERNAL "")
endif()
list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE
  "src/acados_mpc/c_generated_code/**"
  "src/acados_mpc/c_generated_code/**/*"
  "src/acados_mpc/c_generated_code/**/*.*"
)

autoware_package()

find_package(Eigen3 REQUIRED)
find_package(acados REQUIRED)

add_subdirectory(src/acados_mpc)

set(ACADOS_DEPS_INCLUDE_DIRS "")
foreach(_target acados blasfeo hpipm)
  get_target_property(_dirs ${_target} INTERFACE_INCLUDE_DIRECTORIES)
  if(_dirs)
    list(APPEND ACADOS_DEPS_INCLUDE_DIRS ${_dirs})
  endif()
endforeach()
list(REMOVE_DUPLICATES ACADOS_DEPS_INCLUDE_DIRS)
if(ACADOS_DEPS_INCLUDE_DIRS)
  ament_export_include_directories(${ACADOS_DEPS_INCLUDE_DIRS})
endif()

find_package(ament_cmake_export_dependencies REQUIRED)
find_package(ament_cmake_export_link_flags REQUIRED)
ament_export_dependencies(acados)

# Propagate acados link flags so downstream (e.g. static_centerline_generator) that
# links to path_optimizer can resolve libacados transitively without linking to acados explicitly.
get_target_property(_acados_location acados IMPORTED_LOCATION)
if(NOT _acados_location)
  get_target_property(_acados_location acados IMPORTED_LOCATION_RELEASE)
endif()
if(_acados_location)
  get_filename_component(_acados_lib_dir "${_acados_location}" DIRECTORY)
  ament_export_link_flags(-L"${_acados_lib_dir}" -lacados)
else()
  # Fallback when acados is not an imported target (e.g. in-tree)
  set(_acados_lib_dir "$ENV{ACADOS_SOURCE_DIR}/lib")
  if(_acados_lib_dir AND EXISTS "${_acados_lib_dir}")
    ament_export_link_flags(-L"${_acados_lib_dir}" -lacados)
  endif()
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/replan_checker.cpp
  src/mpt_optimizer.cpp
  src/state_equation_generator.cpp
  src/debug_marker.cpp
  src/vehicle_model/vehicle_model_interface.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/utils/trajectory_utils.cpp
  src/utils/geometry_utils.cpp
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} acados_interface)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::path_optimizer::PathOptimizer"
  EXECUTABLE path_optimizer_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} test/test_path_optimizer_node_interface.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE launch config rviz)

install(PROGRAMS scripts/calculation_time_plotter.py DESTINATION lib/${PROJECT_NAME})
install(TARGETS acados_interface LIBRARY DESTINATION lib)
install(FILES src/acados_mpc/include/acados_interface.hpp DESTINATION include/autoware/path_optimizer)

# Downstream (e.g. autoware_trajectory_optimizer) includes acados_interface.hpp -> c_generated_code/...
set(MODEL "curvilinear_bicycle_model_spatial")
set(GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/acados_mpc/c_generated_code")
install(FILES
  ${GEN_DIR}/acados_solver_${MODEL}.h
  ${GEN_DIR}/${MODEL}_model/${MODEL}_model.h
  ${GEN_DIR}/${MODEL}_constraints/${MODEL}_constraints.h
  DESTINATION include/autoware/path_optimizer/c_generated_code
  OPTIONAL
)
