# acados_mpc subdirectory - part of autoware_path_optimizer package

# Use paths set by parent CMakeLists.txt (ACADOS_WORKSPACE_DIR, TERA_RENDERER_SOURCE_DIR)

message(STATUS "Using acados from workspace: ${ACADOS_WORKSPACE_DIR}")

# Build acados in workspace if not already built
get_filename_component(ACADOS_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/acados_install" ABSOLUTE)
get_filename_component(ACADOS_BUILD_DIR "${CMAKE_BINARY_DIR}/acados_build" ABSOLUTE)

set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_PREFIX}/include")
set(ACADOS_LIB_DIR "${ACADOS_INSTALL_PREFIX}/lib")
set(ACADOS_SOURCE_DIR "${ACADOS_WORKSPACE_DIR}")

file(MAKE_DIRECTORY ${ACADOS_INCLUDE_DIR})
file(MAKE_DIRECTORY ${ACADOS_LIB_DIR})

# Configure and build acados from workspace source
# Note: jobserver warnings from acados's internal build are harmless - they occur because
# acados's Makefiles call make recursively without the '+' prefix. This doesn't affect functionality.
add_custom_target(acados_build
    COMMAND ${CMAKE_COMMAND} -E make_directory ${ACADOS_BUILD_DIR}
    COMMAND ${CMAKE_COMMAND} -S ${ACADOS_WORKSPACE_DIR} -B ${ACADOS_BUILD_DIR}
        -DCMAKE_INSTALL_PREFIX=${ACADOS_INSTALL_PREFIX}
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DACADOS_WITH_QPOASES=ON
        -DBUILD_SHARED_LIBS=OFF
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    COMMAND ${CMAKE_COMMAND} --build ${ACADOS_BUILD_DIR} --config ${CMAKE_BUILD_TYPE}
    COMMAND ${CMAKE_COMMAND} --build ${ACADOS_BUILD_DIR} --config ${CMAKE_BUILD_TYPE} --target install
    WORKING_DIRECTORY ${ACADOS_WORKSPACE_DIR}
    COMMENT "Building acados from workspace source..."
)
set(acados_target acados_build)

# --- Define Imported Targets ---
# 1. Standard components (Usually direct in /lib)
foreach(lib acados blasfeo hpipm)
    add_library(${lib} STATIC IMPORTED GLOBAL)
    set_target_properties(${lib} PROPERTIES IMPORTED_LOCATION "${ACADOS_LIB_DIR}/lib${lib}.a")
    add_dependencies(${lib} ${acados_target})
endforeach()

# 2. qpoases - Flexible Path Handling
add_library(qpoases STATIC IMPORTED GLOBAL)

# Check three common locations where acados puts this file
set(QPOASES_PATH_1 "${ACADOS_LIB_DIR}/libqpOASES_e.a")
set(QPOASES_PATH_2 "${ACADOS_LIB_DIR}/libqpoases.a")
set(QPOASES_PATH_3 "${ACADOS_LIB_DIR}/qpoases/libqpoases.a")
set(QPOASES_PATH_4 "${ACADOS_INSTALL_PREFIX}/external/qpoases/lib/libqpoases.a")

# We use the one we found in the install log as the primary choice
set_target_properties(qpoases PROPERTIES IMPORTED_LOCATION "${QPOASES_PATH_1}")
add_dependencies(qpoases ${acados_target})

# --- Build t_renderer from source ---
# t_renderer is built from the tera_renderer Rust source using cargo
set(T_RENDERER_BUILT "${TERA_RENDERER_SOURCE_DIR}/target/release/t_renderer")
set(T_RENDERER_OUTPUT "${ACADOS_SOURCE_DIR}/bin/t_renderer")

# Create bin directory if it doesn't exist
get_filename_component(ACADOS_BIN_DIR "${ACADOS_SOURCE_DIR}/bin" ABSOLUTE)
file(MAKE_DIRECTORY ${ACADOS_BIN_DIR})

# Check if t_renderer binary already exists in output location
if(NOT EXISTS "${T_RENDERER_OUTPUT}")
    # Build from source if cargo is available and tera_renderer source exists
    find_program(CARGO_EXECUTABLE cargo)
    if(CARGO_EXECUTABLE AND EXISTS "${TERA_RENDERER_SOURCE_DIR}/Cargo.toml")
        message(STATUS "Building t_renderer from source using cargo...")
        add_custom_target(build_t_renderer
            COMMAND ${CARGO_EXECUTABLE} build --release
            WORKING_DIRECTORY ${TERA_RENDERER_SOURCE_DIR}
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${T_RENDERER_BUILT}
                ${T_RENDERER_OUTPUT}
            COMMAND chmod +x ${T_RENDERER_OUTPUT}
            COMMENT "Building t_renderer from source..."
        )
        set(t_renderer_target build_t_renderer)
    # Check if already built from source in a previous build
    elseif(EXISTS "${T_RENDERER_BUILT}")
        message(STATUS "Found previously built t_renderer at ${T_RENDERER_BUILT}, copying to ${T_RENDERER_OUTPUT}")
        add_custom_target(build_t_renderer
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${T_RENDERER_BUILT}
                ${T_RENDERER_OUTPUT}
            COMMAND chmod +x ${T_RENDERER_OUTPUT}
            COMMENT "Copying previously built t_renderer..."
        )
        set(t_renderer_target build_t_renderer)
    else()
        # Report why the cargo build path was not taken
        set(T_REASON "")
        if(NOT CARGO_EXECUTABLE)
            set(T_REASON "${T_REASON}  - cargo not found in PATH (install Rust e.g. via rustup)\n")
        endif()
        if(NOT EXISTS "${TERA_RENDERER_SOURCE_DIR}/Cargo.toml")
            set(T_REASON "${T_REASON}  - tera_renderer source missing at ${TERA_RENDERER_SOURCE_DIR}\n"
                "    (run: vcs import src < autoware.repos --recursive)\n")
        endif()
        if(NOT EXISTS "${T_RENDERER_BUILT}")
            set(T_REASON "${T_REASON}  - no pre-built binary at ${T_RENDERER_BUILT}\n")
        endif()
        message(FATAL_ERROR "t_renderer could not be built or found. Reasons:\n${T_REASON}"
            "Fix by: (1) ensuring cargo is in PATH and Rust is up to date (rustup update stable), "
            "(2) importing tera_renderer via vcs, or (3) building t_renderer elsewhere and placing it at ${T_RENDERER_BUILT}")
        add_custom_target(build_t_renderer
            COMMENT "t_renderer cannot be built"
        )
        set(t_renderer_target build_t_renderer)
    endif()
else()
    message(STATUS "t_renderer already exists at ${T_RENDERER_OUTPUT}")
    add_custom_target(build_t_renderer
        COMMENT "t_renderer already exists"
    )
    set(t_renderer_target build_t_renderer)
endif()

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

# Install static libraries for downstream packages
install(FILES
  "${ACADOS_LIB_DIR_ABS}/libacados.a"
  "${ACADOS_LIB_DIR_ABS}/libhpipm.a"
  "${ACADOS_LIB_DIR_ABS}/libblasfeo.a"
  "${ACADOS_LIB_DIR_ABS}/libqpOASES_e.a"
  DESTINATION lib
  OPTIONAL
)
