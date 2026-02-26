# acados_mpc - MPC solver using acados
# Requires acados installed at /opt/acados (via ansible-playbook autoware.dev_env.setup_acados)
# and .bashrc exports: CMAKE_PREFIX_PATH, ACADOS_SOURCE_DIR, LD_LIBRARY_PATH

set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_SOURCE_DIR)
    set(ACADOS_SOURCE_DIR "/opt/acados")
endif()

# Validate required acados components
foreach(_check "bin/t_renderer" ".venv/bin/python3" "interfaces/acados_template")
    if(NOT EXISTS "${ACADOS_SOURCE_DIR}/${_check}")
        message(FATAL_ERROR "${_check} not found in ${ACADOS_SOURCE_DIR}. "
          "Run: ansible-playbook autoware.dev_env.setup_acados --ask-become-pass")
    endif()
endforeach()

# --- Code generation (Python → C via t_renderer) ---
set(MODEL "curvilinear_bicycle_model_spatial")
set(GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code")

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
set_source_files_properties(${GENERATED_MPC_FILES} PROPERTIES GENERATED TRUE)

set(_template_path "${ACADOS_SOURCE_DIR}/interfaces/acados_template")
set(_env_vars
  "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
  "PYTHONPATH=$ENV{PYTHONPATH}:${_template_path}"
)

add_custom_command(
  OUTPUT ${GENERATED_MPC_FILES}
  COMMAND ${CMAKE_COMMAND} -E env ${_env_vars}
  ${ACADOS_SOURCE_DIR}/.venv/bin/python3
  -m generators.path_tracking_mpc_spatial_with_body_points
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_spatial_with_body_points.py
  COMMENT "Generating MPC solver code..."
  VERBATIM
)

# --- acados_interface library ---
add_library(acados_interface SHARED
  src/acados_interface.cpp
  ${GENERATED_MPC_FILES}
)

target_include_directories(acados_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${ACADOS_SOURCE_DIR}/include
  ${ACADOS_SOURCE_DIR}/include/acados
  ${ACADOS_SOURCE_DIR}/include/acados_c
  ${ACADOS_SOURCE_DIR}/include/blasfeo/include
  ${ACADOS_SOURCE_DIR}/include/hpipm/include
)

target_link_libraries(acados_interface PRIVATE acados)

# Suppress warnings on generated C code
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
    set_source_files_properties(${GENERATED_MPC_FILES} PROPERTIES
      COMPILE_OPTIONS "-Wno-all;-Wno-extra;-Wno-unused-function")
endif()
