# --- acados_mpc - MPC solver using acados ---

# 1. Environment and Path Setup
set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_SOURCE_DIR)
    set(ACADOS_SOURCE_DIR "/opt/acados")
endif()

# 2. Code generation (Python → C via t_renderer)
# Generate into binary dir so source tree is never modified; stub header stays in source for clang-tidy/IDE.
set(MODEL "curvilinear_bicycle_model_spatial")
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/c_generated_code")

set(GENERATED_MPC_FILES
  ${GEN_DIR}/acados_solver_${MODEL}.h
  ${GEN_DIR}/acados_solver_${MODEL}.c
  ${GEN_DIR}/${MODEL}_model/${MODEL}_expl_ode_fun.c
  ${GEN_DIR}/${MODEL}_model/${MODEL}_expl_vde_adj.c
  ${GEN_DIR}/${MODEL}_model/${MODEL}_expl_vde_forw.c
  ${GEN_DIR}/${MODEL}_model/${MODEL}_model.h
  ${GEN_DIR}/${MODEL}_constraints/${MODEL}_constr_h_fun.c
  ${GEN_DIR}/${MODEL}_constraints/${MODEL}_constr_h_fun_jac_uxt_zt.c
  ${GEN_DIR}/${MODEL}_constraints/${MODEL}_constraints.h
)

# CRITICAL: Mark files as generated so CMake knows to wait for them
set_source_files_properties(${GENERATED_MPC_FILES} PROPERTIES GENERATED TRUE)
# Get the parent directory of 'generators' so 'from generators.xxx' works
get_filename_component(PACKAGE_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)

set(_template_path "${ACADOS_SOURCE_DIR}/interfaces/acados_template")

set(_env_vars
  "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
  "PYTHONPATH=${_template_path}:${PACKAGE_ROOT_DIR}:$ENV{PYTHONPATH}"
)

# 3. Define the Generation Target (generator writes to CWD/c_generated_code → binary dir)
add_custom_command(
  OUTPUT ${GENERATED_MPC_FILES}
  COMMAND ${CMAKE_COMMAND} -E env ${_env_vars}
    ${ACADOS_SOURCE_DIR}/.venv/bin/python3
    ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_spatial_with_body_points.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_spatial_with_body_points.py
  COMMENT "Generating acados MPC solver code"
)

# Create a logical target for the generation to ensure proper dependency tracking
add_custom_target(generate_acados_code DEPENDS ${GENERATED_MPC_FILES})

# 4. acados_interface library
add_library(acados_interface SHARED
  src/acados_interface.cpp
  ${GENERATED_MPC_FILES}
)

# Ensure the library waits for the generator to finish
add_dependencies(acados_interface generate_acados_code)

# Binary dir first so real generated header shadows stub; source dir for clang-tidy/IDE when no build.
target_include_directories(acados_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${ACADOS_SOURCE_DIR}/include
  ${ACADOS_SOURCE_DIR}/include/acados
  ${ACADOS_SOURCE_DIR}/include/acados_c
  ${ACADOS_SOURCE_DIR}/include/blasfeo/include
  ${ACADOS_SOURCE_DIR}/include/hpipm/include
)

target_link_libraries(acados_interface PUBLIC acados)

# 5. Suppress warnings on generated C code
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${GENERATED_MPC_FILES} PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-parameter;-Wno-unused-function")
endif()
