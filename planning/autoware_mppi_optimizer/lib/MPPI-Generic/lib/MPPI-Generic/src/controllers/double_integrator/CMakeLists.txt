set(LIBRARY_NAME double_integrator_mppi)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)

add_library(${LIBRARY_NAME} SHARED
        double_integrator_mppi.cu
        )

target_link_libraries( ${LIBRARY_NAME}
        ${YAML_CPP_LIBRARIES}
        ${MPPI_HEADER_LIBRARY_NAME}
        )

SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

install(TARGETS ${LIBRARY_NAME}
        # IMPORTANT: Add the library to the "export-set"
        EXPORT ${PROJECT_NAME}-targets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/${CMAKE_PROJECT_NAME}
        )
