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
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(acados REQUIRED)

# --- acados MPC solver (builds acados_interface library + code generation) ---
add_subdirectory(src/acados_mpc)

# --- Main package library ---
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
