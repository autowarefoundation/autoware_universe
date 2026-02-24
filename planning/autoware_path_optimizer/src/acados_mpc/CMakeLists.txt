# acados_mpc subdirectory - part of autoware_path_optimizer package
# acados and t_renderer are installed at setup time via the collection playbook:
#   ansible-playbook autoware.dev_env.setup_acados --ask-become-pass
# (See ansible/roles/acados/README.md; installs to /opt/acados, t_renderer at /opt/acados/bin/t_renderer.)

# Prefer explicit prefix, else detect from common locations.
# acados is typically installed as shared libs (.so); we prefer .so then fall back to .a.
set(ACADOS_INSTALL_PREFIX "" CACHE PATH "acados install prefix (leave empty to auto-detect)")
set(ACADOS_LIB_ACADOS_PATH "")
if(ACADOS_INSTALL_PREFIX)
    if(EXISTS "${ACADOS_INSTALL_PREFIX}/lib/libacados.so")
        set(ACADOS_LIB_ACADOS_PATH "${ACADOS_INSTALL_PREFIX}/lib/libacados.so")
    elseif(EXISTS "${ACADOS_INSTALL_PREFIX}/lib/libacados.a")
        set(ACADOS_LIB_ACADOS_PATH "${ACADOS_INSTALL_PREFIX}/lib/libacados.a")
    endif()
endif()
if(NOT ACADOS_LIB_ACADOS_PATH)
    if(EXISTS "/opt/acados/lib/libacados.so")
        set(ACADOS_INSTALL_PREFIX "/opt/acados")
        set(ACADOS_LIB_ACADOS_PATH "/opt/acados/lib/libacados.so")
    elseif(EXISTS "/opt/acados/lib/libacados.a")
        set(ACADOS_INSTALL_PREFIX "/opt/acados")
        set(ACADOS_LIB_ACADOS_PATH "/opt/acados/lib/libacados.a")
    elseif(EXISTS "/usr/local/lib/libacados.so")
        set(ACADOS_INSTALL_PREFIX "/usr/local")
        set(ACADOS_LIB_ACADOS_PATH "/usr/local/lib/libacados.so")
    elseif(EXISTS "/usr/local/lib/libacados.a")
        set(ACADOS_INSTALL_PREFIX "/usr/local")
        set(ACADOS_LIB_ACADOS_PATH "/usr/local/lib/libacados.a")
    else()
        message(FATAL_ERROR "Pre-built acados not found. Looked for libacados.a or libacados.so in /opt/acados/lib and /usr/local/lib. "
            "Run: ansible-playbook autoware.dev_env.setup_acados --ask-become-pass (after ansible-galaxy collection install). "
            "If acados is elsewhere, set -DACADOS_INSTALL_PREFIX=/path/to/acados.")
    endif()
endif()

# Source tree for Python templates and t_renderer (ansible clones to /opt/acados)
if(EXISTS "/opt/acados/bin/t_renderer" OR EXISTS "/opt/acados/interfaces/acados_template")
    set(ACADOS_SOURCE_DIR "/opt/acados")
else()
    set(ACADOS_SOURCE_DIR "${ACADOS_INSTALL_PREFIX}")
endif()
message(STATUS "Using pre-built acados from ${ACADOS_INSTALL_PREFIX}")

add_custom_target(acados_prebuilt COMMENT "Using pre-built acados from setup")
set(acados_target acados_prebuilt)

set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_PREFIX}/include")
set(ACADOS_LIB_DIR "${ACADOS_INSTALL_PREFIX}/lib")
if(ACADOS_LIB_ACADOS_PATH MATCHES "\\.so$")
    set(ACADOS_LIB_TYPE SHARED)
    set(ACADOS_LIB_SUFFIX ".so")
else()
    set(ACADOS_LIB_TYPE STATIC)
    set(ACADOS_LIB_SUFFIX ".a")
endif()

# --- Define Imported Targets (idempotent for reconfigures) ---
# 1. Standard components (acados, blasfeo, hpipm - may be .a or .so)
if(NOT TARGET acados)
    add_library(acados ${ACADOS_LIB_TYPE} IMPORTED GLOBAL)
    set_target_properties(acados PROPERTIES IMPORTED_LOCATION "${ACADOS_LIB_ACADOS_PATH}")
endif()
add_dependencies(acados ${acados_target})
foreach(lib blasfeo hpipm)
    if(NOT TARGET ${lib})
        add_library(${lib} ${ACADOS_LIB_TYPE} IMPORTED GLOBAL)
        set_target_properties(${lib} PROPERTIES IMPORTED_LOCATION "${ACADOS_LIB_DIR}/lib${lib}${ACADOS_LIB_SUFFIX}")
    endif()
    add_dependencies(${lib} ${acados_target})
endforeach()

# 2. qpoases - try common names and .a/.so
set(QPOASES_PATH_1 "${ACADOS_LIB_DIR}/libqpOASES_e${ACADOS_LIB_SUFFIX}")
set(QPOASES_PATH_2 "${ACADOS_LIB_DIR}/libqpoases${ACADOS_LIB_SUFFIX}")
set(QPOASES_PATH_3 "${ACADOS_LIB_DIR}/qpoases/libqpoases${ACADOS_LIB_SUFFIX}")
if(EXISTS "${QPOASES_PATH_1}")
    set(QPOASES_LOCATION "${QPOASES_PATH_1}")
elseif(EXISTS "${QPOASES_PATH_2}")
    set(QPOASES_LOCATION "${QPOASES_PATH_2}")
elseif(EXISTS "${QPOASES_PATH_3}")
    set(QPOASES_LOCATION "${QPOASES_PATH_3}")
else()
    set(QPOASES_LOCATION "${QPOASES_PATH_1}")
endif()
if(NOT TARGET qpoases)
    add_library(qpoases ${ACADOS_LIB_TYPE} IMPORTED GLOBAL)
    set_target_properties(qpoases PROPERTIES IMPORTED_LOCATION "${QPOASES_LOCATION}")
endif()
add_dependencies(qpoases ${acados_target})

# t_renderer is installed at setup time (autoware.dev_env.setup_acados) to /opt/acados/bin/t_renderer
set(T_RENDERER_OUTPUT "${ACADOS_SOURCE_DIR}/bin/t_renderer")
if(NOT EXISTS "${T_RENDERER_OUTPUT}")
    message(FATAL_ERROR "Pre-built t_renderer not found at ${T_RENDERER_OUTPUT}. "
        "Run: ansible-playbook autoware.dev_env.setup_acados --ask-become-pass")
endif()
message(STATUS "Using pre-built t_renderer at ${T_RENDERER_OUTPUT}")
add_custom_target(build_t_renderer COMMENT "Using pre-built t_renderer from setup")
set(t_renderer_target build_t_renderer)

# --- Autoware Setup ---
# Set environment variables for Python script to find acados source and template
set(ACADOS_TEMPLATE_PATH "${ACADOS_SOURCE_DIR}/interfaces/acados_template")
set(ACADOS_SOURCE_DIR_ENV "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}")
# Preserve existing PYTHONPATH if set, otherwise just set it to acados_template path
if(DEFINED ENV{PYTHONPATH})
    set(ACADOS_PYTHONPATH_ENV "PYTHONPATH=$ENV{PYTHONPATH}:${ACADOS_TEMPLATE_PATH}")
else()
    set(ACADOS_PYTHONPATH_ENV "PYTHONPATH=${ACADOS_TEMPLATE_PATH}")
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

# --- Generated MPT Files ---
set(GENERATED_MPT_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_solver_curvilinear_bicycle_model_spatial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_solver_curvilinear_bicycle_model_spatial.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_expl_ode_fun.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_expl_vde_adj.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_expl_vde_forw.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_model.h
    # Nonlinear path constraints (h): present when NH>0
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_constraints/curvilinear_bicycle_model_spatial_constr_h_fun.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_constraints/curvilinear_bicycle_model_spatial_constr_h_fun_jac_uxt_zt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_constraints/curvilinear_bicycle_model_spatial_constraints.h
)
set_source_files_properties(${GENERATED_MPT_FILES} PROPERTIES GENERATED TRUE)

add_custom_command(
    OUTPUT ${GENERATED_MPT_FILES}
    COMMAND ${CMAKE_COMMAND} -E env "${ACADOS_SOURCE_DIR_ENV}" "${ACADOS_PYTHONPATH_ENV}"
            python3 -m generators.path_tracking_mpc_spatial_with_body_points
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_spatial_with_body_points.py
    COMMENT "Generating MPT code..."
    VERBATIM
)
add_custom_target(generate_mpt DEPENDS ${GENERATED_MPT_FILES})
add_dependencies(generate_mpt ${acados_target} ${t_renderer_target})

# --- acados_interface ---
add_library(acados_interface SHARED
  src/acados_interface.cpp
  ${GENERATED_MPT_FILES}
)

target_include_directories(acados_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/acados>
  $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/acados_c>
  $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/blasfeo/include>
  $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/hpipm/include>
  $<INSTALL_INTERFACE:include/blasfeo/include>
  $<INSTALL_INTERFACE:include/hpipm/include>
)

target_link_libraries(acados_interface PRIVATE acados hpipm blasfeo qpoases)
add_dependencies(acados_interface generate_mpt)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
    set_source_files_properties(${GENERATED_MPT_FILES} PROPERTIES COMPILE_OPTIONS "-Wno-all;-Wno-extra;-Wno-unused-function")
endif()

# Install acados headers and libraries to make the package hermetic
get_filename_component(ACADOS_INCLUDE_DIR_ABS "${ACADOS_INCLUDE_DIR}" ABSOLUTE)
get_filename_component(ACADOS_LIB_DIR_ABS "${ACADOS_LIB_DIR}" ABSOLUTE)

# Install headers (preserves directory structure for acados/utils/types.h)
foreach(header_dir acados acados_c hpipm blasfeo)
  install(DIRECTORY "${ACADOS_INCLUDE_DIR_ABS}/${header_dir}"
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )
endforeach()

# Install acados libraries for downstream packages (same type as found: .a or .so)
install(FILES
  "${ACADOS_LIB_ACADOS_PATH}"
  "${ACADOS_LIB_DIR_ABS}/libhpipm${ACADOS_LIB_SUFFIX}"
  "${ACADOS_LIB_DIR_ABS}/libblasfeo${ACADOS_LIB_SUFFIX}"
  "${QPOASES_LOCATION}"
  DESTINATION lib
  OPTIONAL
)
