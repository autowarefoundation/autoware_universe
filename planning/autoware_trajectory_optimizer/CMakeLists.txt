cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_optimizer)

find_package(autoware_cmake REQUIRED)
find_package(acados REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

autoware_package()

find_package(ament_cmake_export_dependencies REQUIRED)
ament_export_dependencies(acados)

# Temporal kinematic-bicycle acados solver: Python codegen at build time (add_custom_command).
# See src/acados_mpt/CMakeLists.txt — target acados_interface_temporal.
add_subdirectory(src/acados_mpt)
pluginlib_export_plugin_description_file(autoware_trajectory_optimizer plugins.xml)

ament_auto_add_library(autoware_trajectory_optimizer_component SHARED
  src/trajectory_optimizer.cpp
  src/utils.cpp
)

rclcpp_components_register_node(autoware_trajectory_optimizer_component
  PLUGIN "autoware::trajectory_optimizer::TrajectoryOptimizer"
  EXECUTABLE autoware_trajectory_optimizer_node
)

ament_auto_add_library(autoware_trajectory_optimizer_plugins SHARED
  src/trajectory_optimizer_plugins/trajectory_eb_smoother_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_extender.cpp
  src/trajectory_optimizer_plugins/trajectory_kinematic_feasibility_enforcer.cpp
  src/trajectory_optimizer_plugins/trajectory_mpt_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_temporal_mpt_optimizer.cpp
  src/trajectory_optimizer_plugins/trajectory_point_fixer.cpp
  src/trajectory_optimizer_plugins/trajectory_qp_smoother.cpp
  src/trajectory_optimizer_plugins/trajectory_spline_smoother.cpp
  src/trajectory_optimizer_plugins/trajectory_velocity_optimizer.cpp
  src/trajectory_optimizer_plugins/plugin_utils/continuous_jerk_smoother.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_extender_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_mpt_optimizer_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_temporal_mpt_optimizer_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_point_fixer_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_spline_smoother_utils.cpp
  src/trajectory_optimizer_plugins/plugin_utils/trajectory_velocity_optimizer_utils.cpp
)
target_link_libraries(
  autoware_trajectory_optimizer_plugins
  acados_interface_temporal
  tf2_geometry_msgs::tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)
  ament_add_ros_isolated_gtest(test_autoware_trajectory_optimizer
    ${TEST_SOURCES}
  )
  target_link_libraries(test_autoware_trajectory_optimizer
    autoware_trajectory_optimizer_component
    autoware_trajectory_optimizer_plugins
  )
endif()

install(TARGETS acados_interface_temporal LIBRARY DESTINATION lib)

set(TEMPORAL_MODEL "kinematic_bicycle_temporal")
set(TEMPORAL_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/src/acados_mpt/c_generated_code")
install(FILES
  ${TEMPORAL_GEN_DIR}/acados_solver_${TEMPORAL_MODEL}.h
  ${TEMPORAL_GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_model.h
  ${TEMPORAL_GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constraints.h
  DESTINATION include/autoware/trajectory_optimizer/c_generated_code
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
