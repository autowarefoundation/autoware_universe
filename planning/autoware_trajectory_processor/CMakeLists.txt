cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_processor)

find_package(autoware_cmake REQUIRED)
find_package(acados REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

autoware_package()

find_package(ament_cmake_export_dependencies REQUIRED)
ament_export_dependencies(acados)

# Temporal kinematic-bicycle acados solver: Python codegen at build time (add_custom_command).
# See src/acados_mpt/CMakeLists.txt — target acados_interface_temporal.
add_subdirectory(src/acados_mpt)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

generate_parameter_library(trajectory_processor_param
  param/trajectory_processor_parameter_struct.yaml
)

ament_auto_add_library(autoware_trajectory_processor_utils SHARED
  src/utils.cpp
)
autoware_agnocast_wrapper_setup(autoware_trajectory_processor_utils)

ament_auto_add_library(autoware_trajectory_processor_component SHARED
  src/trajectory_processor.cpp
)

target_link_libraries(autoware_trajectory_processor_component
  autoware_trajectory_processor_utils
  trajectory_processor_param
)

target_compile_options(autoware_trajectory_processor_component PUBLIC
  -Wno-error=deprecated-declarations
)

autoware_agnocast_wrapper_register_node(autoware_trajectory_processor_component
  PLUGIN "autoware::trajectory_processor::TrajectoryProcessor"
  EXECUTABLE autoware_trajectory_processor_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

ament_auto_add_library(autoware_trajectory_modifier_utils SHARED
  src/trajectory_modifier_utils/utils.cpp
  src/trajectory_modifier_utils/obstacle_stop_utils.cpp
)
autoware_agnocast_wrapper_setup(autoware_trajectory_modifier_utils)

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
autoware_agnocast_wrapper_setup(autoware_trajectory_optimizer_plugins)
target_link_libraries(
  autoware_trajectory_optimizer_plugins
  acados_interface_temporal
  autoware_trajectory_processor_utils
  tf2_geometry_msgs::tf2_geometry_msgs
  trajectory_processor_param
)

ament_auto_add_library(autoware_trajectory_modifier_plugins SHARED
  src/trajectory_modifier_plugins/stop_point_fixer.cpp
  src/trajectory_modifier_plugins/obstacle_stop.cpp
  src/trajectory_modifier_plugins/velocity_modifier.cpp
  src/trajectory_modifier_plugins/surround_obstacle_stop.cpp
  src/trajectory_modifier_plugins/traffic_light_stop.cpp
)
autoware_agnocast_wrapper_setup(autoware_trajectory_modifier_plugins)

target_link_libraries(autoware_trajectory_modifier_plugins
  autoware_trajectory_modifier_utils
  trajectory_processor_param
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(NODE_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_obstacle_stop_integration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_stop_point_fixer_integration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_surround_obstacle_stop_integration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_traffic_light_stop_integration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_trajectory_processor_integration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_trajectory_processor_plugin_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_velocity_modifier_integration.cpp
  )

  file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)
  list(REMOVE_ITEM TEST_SOURCES ${NODE_TEST_SOURCES})

  ament_add_ros_isolated_gtest(test_autoware_trajectory_processor
    ${TEST_SOURCES}
  )
  target_link_libraries(test_autoware_trajectory_processor
    autoware_trajectory_processor_component
    autoware_trajectory_optimizer_plugins
    autoware_trajectory_modifier_plugins
    autoware_trajectory_modifier_utils
  )

  if(NOT (DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
    ament_add_ros_isolated_gtest(test_autoware_trajectory_processor_nodes
      ${NODE_TEST_SOURCES}
    )
    target_link_libraries(test_autoware_trajectory_processor_nodes
      autoware_trajectory_processor_component
      autoware_trajectory_optimizer_plugins
      autoware_trajectory_modifier_plugins
      autoware_trajectory_modifier_utils
    )
  endif()
endif()

install(TARGETS acados_interface_temporal LIBRARY DESTINATION lib)

set(TEMPORAL_MODEL "kinematic_bicycle_temporal")
set(TEMPORAL_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/src/acados_mpt/c_generated_code")
install(FILES
  ${TEMPORAL_GEN_DIR}/acados_solver_${TEMPORAL_MODEL}.h
  ${TEMPORAL_GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_model.h
  ${TEMPORAL_GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constraints.h
  DESTINATION include/autoware/trajectory_processor/c_generated_code
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
