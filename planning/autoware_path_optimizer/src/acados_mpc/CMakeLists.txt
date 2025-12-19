# acados_mpc subdirectory - part of autoware_path_optimizer package

include(ExternalProject)

# --- Configuration Options ---
option(ACADOS_ROOT "Path to local acados installation (leave empty to build from source)" "")

if(ACADOS_ROOT)
    message(STATUS "Using local acados installation at: ${ACADOS_ROOT}")
    set(ACADOS_INCLUDE_DIR "${ACADOS_ROOT}/include")
    set(ACADOS_LIB_DIR "${ACADOS_ROOT}/lib")
    add_custom_target(acados_target)
else()
    message(STATUS "Building acados from source...")
    get_filename_component(ACADOS_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/acados_install" ABSOLUTE)
    get_filename_component(ACADOS_BUILD_DIR "${CMAKE_BINARY_DIR}/acados_build" ABSOLUTE)
    
    set(ACADOS_INCLUDE_DIR "${ACADOS_INSTALL_PREFIX}/include")
    set(ACADOS_LIB_DIR "${ACADOS_INSTALL_PREFIX}/lib")

    file(MAKE_DIRECTORY ${ACADOS_INCLUDE_DIR})
    file(MAKE_DIRECTORY ${ACADOS_LIB_DIR})

    include(ProcessorCount)
    ProcessorCount(N)
    if(N EQUAL 0)
        set(ACADOS_BUILD_JOBS 4)
    else()
        set(ACADOS_BUILD_JOBS ${N})
    endif()

    ExternalProject_Add(acados_external
        GIT_REPOSITORY https://github.com/acados/acados.git
        GIT_TAG main
        GIT_SUBMODULES_RECURSE TRUE
        PREFIX "${ACADOS_BUILD_DIR}"
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${ACADOS_INSTALL_PREFIX}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DACADOS_WITH_QPOASES=ON
            -DBUILD_SHARED_LIBS=OFF
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        BUILD_COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE} -j${ACADOS_BUILD_JOBS}
        INSTALL_COMMAND ${CMAKE_COMMAND} --build . --config ${CMAKE_BUILD_TYPE} --target install
        UPDATE_COMMAND ""
        LOG_DOWNLOAD ON LOG_CONFIGURE ON LOG_BUILD ON LOG_INSTALL ON
    )
    set(acados_target acados_external)
endif()

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

# --- Autoware Setup ---
if(NOT ACADOS_ROOT)
    set(ACADOS_SOURCE_DIR_ENV "ACADOS_SOURCE_DIR=${ACADOS_BUILD_DIR}/src/acados_external")
else()
    set(ACADOS_SOURCE_DIR_ENV "ACADOS_UNUSED=1")
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
)
set_source_files_properties(${GENERATED_MPT_FILES} PROPERTIES GENERATED TRUE)

add_custom_command(
    OUTPUT ${GENERATED_MPT_FILES}
    COMMAND ${CMAKE_COMMAND} -E env "${ACADOS_SOURCE_DIR_ENV}"
            python3 ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_spatial_with_body_points.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_spatial_with_body_points.py
    COMMENT "Generating MPT code..."
    VERBATIM
)
add_custom_target(generate_mpt DEPENDS ${GENERATED_MPT_FILES})
if(NOT ACADOS_ROOT)
    add_dependencies(generate_mpt ${acados_target})
endif()

# --- acados_generated_code ---
add_library(acados_generated_code STATIC ${GENERATED_MPT_FILES})
set_target_properties(acados_generated_code PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(acados_generated_code PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/acados>
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/acados_c>
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/blasfeo/include>
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}/hpipm/include>
)
# Note the order: core acados first, then its dependencies
target_link_libraries(acados_generated_code PUBLIC acados hpipm blasfeo qpoases)
add_dependencies(acados_generated_code generate_mpt)

# --- acados_interface ---
add_library(acados_interface src/acados_interface.cpp)
target_include_directories(acados_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${ACADOS_INCLUDE_DIR}>
)
target_link_libraries(acados_interface PUBLIC acados_generated_code acados hpipm blasfeo qpoases)
add_dependencies(acados_interface acados_generated_code)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
    target_compile_options(acados_generated_code PRIVATE -Wno-all -Wno-extra -Wno-unused-function)
endif()