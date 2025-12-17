# acados_mpc subdirectory - part of autoware_path_optimizer package

include_directories(/home/arjunram/Workspace/acados/include/hpipm/include)
include_directories(/home/arjunram/Workspace/acados/include/blasfeo/include)

include_directories(/home/arjunram/Workspace/acados/include)
include_directories(/home/arjunram/Workspace/acados/include/acados)
include_directories(/home/arjunram/Workspace/acados/include/acados_c)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add Autoware install include path so headers like autoware_internal_debug_msgs are found (provided by user workspace)
include_directories(/home/arjunram/Workspace/autoware-MPT/install/autoware_internal_debug_msgs/include)
include_directories(/home/arjunram/Workspace/autoware-MPT/install/autoware_internal_debug_msgs/include/autoware_internal_debug_msgs)
include_directories(/home/arjunram/Workspace/autoware-MPT/install/autoware_planning_msgs/include)
include_directories(/home/arjunram/Workspace/autoware-MPT/install/autoware_planning_msgs/include/autoware_planning_msgs)

add_library(acados_interface
    include/acados_interface.hpp
    src/acados_interface.cpp
)

# Make acados_interface headers available to consumers
target_include_directories(acados_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/c_generated_code>
    /home/arjunram/Workspace/acados/include
    /home/arjunram/Workspace/acados/include/acados
    /home/arjunram/Workspace/acados/include/acados_c
    /home/arjunram/Workspace/acados/include/blasfeo/include
    /home/arjunram/Workspace/acados/include/hpipm/include
)

# Ensure acados_interface is built after the generated files exist
add_dependencies(acados_interface generate_mpt)

# Ensure compiler waits for generated header; declare it as a generated source
set(GEN_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_solver_curvilinear_bicycle_model_spatial.h)
set_source_files_properties(${GEN_HEADER} PROPERTIES GENERATED TRUE)
target_sources(acados_interface PRIVATE ${GEN_HEADER})

add_library(acados_generated_code
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_solver_curvilinear_bicycle_model_spatial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_solver_curvilinear_bicycle_model_spatial.c
)

# Ensure headers in c_generated_code are visible to consumers
target_include_directories(acados_generated_code PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code>
    $<INSTALL_INTERFACE:include/c_generated_code>
    /home/arjunram/Workspace/acados/include
    /home/arjunram/Workspace/acados/include/acados
    /home/arjunram/Workspace/acados/include/acados_c
    /home/arjunram/Workspace/acados/include/blasfeo/include
    /home/arjunram/Workspace/acados/include/hpipm/include
)

# Generated code may produce unused-variable/parameter warnings; don't treat them as errors
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
    target_compile_options(acados_generated_code PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-Wno-error>
        $<$<COMPILE_LANGUAGE:C>:-Wno-unused-parameter>
        $<$<COMPILE_LANGUAGE:C>:-Wno-unused-variable>
        $<$<COMPILE_LANGUAGE:C>:-Wno-unused-function>
    )
endif()

# Generated files produced by generate_mpt.sh
set(GENERATED_MPT_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_solver_curvilinear_bicycle_model_spatial.h
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_solver_curvilinear_bicycle_model_spatial.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_expl_ode_fun.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_expl_vde_adj.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_expl_vde_forw.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_model.h
)

# Mark all generated files as GENERATED so CMake knows they're build artifacts
set_source_files_properties(${GENERATED_MPT_FILES} PROPERTIES GENERATED TRUE)

# Run generator script to produce MPT files before compiling generated code
add_custom_command(
    OUTPUT ${GENERATED_MPT_FILES}
    COMMAND ${CMAKE_COMMAND} -E echo "Running path_tracking_mpc_spatial_with_body_points.py"
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_spatial_with_body_points.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generators/path_tracking_mpc_spatial_with_body_points.py
    COMMENT "Generating MPT code with path_tracking_mpc_spatial_with_body_points.py"
    VERBATIM
)

add_custom_target(generate_mpt DEPENDS ${GENERATED_MPT_FILES})

# Ensure generated code target depends on generated MPT files
add_dependencies(acados_generated_code generate_mpt)

# Make sure the generated-code target can find acados headers
target_include_directories(acados_generated_code
    PUBLIC
    /home/arjunram/Workspace/acados/include
    /home/arjunram/Workspace/acados/include/acados
    /home/arjunram/Workspace/acados/include/acados_c
    /home/arjunram/Workspace/acados/include/blasfeo/include
    /home/arjunram/Workspace/acados/include/hpipm/include
)

# also include the generated model external function sources
target_sources(acados_generated_code PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_expl_ode_fun.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_expl_vde_adj.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_expl_vde_forw.c
    ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/curvilinear_bicycle_model_spatial_model/curvilinear_bicycle_model_spatial_model.h
)

# Make the list of generated files available as a target property for downstream consumers
set_target_properties(acados_generated_code PROPERTIES
    GENERATED_MPT_FILES "${GENERATED_MPT_FILES}"
)

# Link acados_generated_code against the precompiled Acados libraries
# Use absolute paths to .so files for reliable linking
target_link_libraries(acados_generated_code
    /home/arjunram/Workspace/acados/lib/libacados.so
    /home/arjunram/Workspace/acados/lib/libblasfeo.so
    /home/arjunram/Workspace/acados/lib/libhpipm.so
)

target_link_libraries(acados_interface
    acados_generated_code
)
