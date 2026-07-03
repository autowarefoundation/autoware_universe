# --- acados_mpt - temporal kinematic bicycle MPC (acados) ---

# 1. Environment and Path Setup
set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "/opt/acados")
endif()

# 2. Code generation (Python → C via t_renderer)
# Generate into binary dir so source tree is never modified; stub header stays in source for clang-tidy/IDE.
set(TEMPORAL_MODEL kinematic_bicycle_temporal)
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/c_generated_code")

set(GENERATED_TEMPORAL_MPC_FILES
  ${GEN_DIR}/acados_solver_${TEMPORAL_MODEL}.h
  ${GEN_DIR}/acados_solver_${TEMPORAL_MODEL}.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_expl_ode_fun.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_expl_vde_adj.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_expl_vde_forw.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_model/${TEMPORAL_MODEL}_model.h
  ${GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constr_h_fun.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constr_h_fun_jac_uxt_zt.c
  ${GEN_DIR}/${TEMPORAL_MODEL}_constraints/${TEMPORAL_MODEL}_constraints.h
)

# CRITICAL: Mark files as generated so CMake knows to wait for them
set_source_files_properties(${GENERATED_TEMPORAL_MPC_FILES} PROPERTIES GENERATED TRUE)
# acados_mpt on PYTHONPATH so 'from generators...' works
get_filename_component(PACKAGE_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(_template_path "${ACADOS_SOURCE_DIR}/interfaces/acados_template")
set(_env_vars
  "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
  "PYTHONPATH=${_template_path}:${PACKAGE_ROOT_DIR}:$ENV{PYTHONPATH}"
)

# 3. Define the Generation Target (generator writes to CWD/c_generated_code → binary dir)
add_custom_command(
  OUTPUT ${GENERATED_TEMPORAL_MPC_FILES}
  COMMAND ${CMAKE_COMMAND} -E env ${_env_vars}
    ${ACADOS_SOURCE_DIR}/.venv/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_temporal.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_temporal.py
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/bicycle_model_temporal.py
  COMMENT "Generating acados MPC solver code"
)

# Create a logical target for the generation to ensure proper dependency tracking
add_custom_target(generate_acados_code_temporal DEPENDS ${GENERATED_TEMPORAL_MPC_FILES})

# Stage generated headers under include/autoware/trajectory_optimizer/ so installed-style
# includes resolve at build time (matches install DESTINATION layout).
set(STAGING_INCLUDE_ROOT "${CMAKE_CURRENT_BINARY_DIR}/include")
set(STAGING_GEN_LINK "${STAGING_INCLUDE_ROOT}/autoware/trajectory_optimizer/c_generated_code")
set(STAGING_GEN_STAMP "${CMAKE_CURRENT_BINARY_DIR}/staged_c_generated_code.stamp")
add_custom_command(
  OUTPUT ${STAGING_GEN_STAMP}
  COMMAND ${CMAKE_COMMAND} -E make_directory
    "${STAGING_INCLUDE_ROOT}/autoware/trajectory_optimizer"
  COMMAND ${CMAKE_COMMAND} -E rm -rf "${STAGING_GEN_LINK}"
  COMMAND ${CMAKE_COMMAND} -E create_symlink "${GEN_DIR}" "${STAGING_GEN_LINK}"
  COMMAND ${CMAKE_COMMAND} -E touch "${STAGING_GEN_STAMP}"
  DEPENDS ${GENERATED_TEMPORAL_MPC_FILES}
  COMMENT "Staging acados generated headers for install-layout includes"
)
add_custom_target(stage_acados_gen_includes DEPENDS ${STAGING_GEN_STAMP})

# 4. acados_interface_temporal library
add_library(acados_interface_temporal SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/acados_interface.cpp
  ${GENERATED_TEMPORAL_MPC_FILES}
)

# Ensure the library waits for the generator to finish
add_dependencies(
  acados_interface_temporal generate_acados_code_temporal stage_acados_gen_includes
)

target_include_directories(acados_interface_temporal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<BUILD_INTERFACE:${STAGING_INCLUDE_ROOT}>
  $<INSTALL_INTERFACE:include>
  ${ACADOS_SOURCE_DIR}/include
  ${ACADOS_SOURCE_DIR}/include/acados
  ${ACADOS_SOURCE_DIR}/include/acados_c
  ${ACADOS_SOURCE_DIR}/include/blasfeo/include
  ${ACADOS_SOURCE_DIR}/include/hpipm/include
)

target_link_libraries(acados_interface_temporal PUBLIC acados)

# 5. Suppress warnings on generated C code
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${GENERATED_TEMPORAL_MPC_FILES} PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-parameter;-Wno-unused-function")
endif()
