# Add project includes and cuda includes for g++

#include test utils
include_directories(include)

# Create header file pointing to test neural network
set(HEADER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/templated_headers/)

# Create header file pointing to test map
set(TEST_MAP_FOLDER "${PROJECT_BINARY_DIR}")
set(TEST_NETWORK_FOLDER "${PROJECT_BINARY_DIR}")

if(MPPI_BUILD_TESTS)
    message("")
    find_package(PythonInterp)

    # Create neural network map
    execute_process(COMMAND ${PYTHON_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/scripts/autorally/test/generateTestNetwork.py
            -o ${TEST_NETWORK_FOLDER}
            OUTPUT_FILE /dev/null
            )
    message(STATUS "Created ${TEST_NETWORK_FOLDER}/*.npz")
    message("")
endif()

if(MPPI_BUILD_TESTS)
    message("")
    find_package(PythonInterp)

    # Create test map
    execute_process(COMMAND ${PYTHON_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/scripts/autorally/test/generateTestMaps.py
            -o ${TEST_MAP_FOLDER}/
            OUTPUT_FILE /dev/null
            )
    message(STATUS "Created folder ${TEST_MAP_FOLDER}/*.npz")
    message("")
endif()


file(GLOB HEADERS ${HEADER_PATH}*)
foreach(T_FILE IN LISTS HEADERS)
    get_filename_component(T_NAME ${T_FILE} NAME_WE)
    string(REPLACE ".in" "" STRIPPED_NAME ${T_NAME})
    message("stripped name ${STRIPPED_NAME}")
    set(TEST_NETWORK_HEADER_FILE ${PROJECT_BINARY_DIR}/include/${STRIPPED_NAME}.h)
    configure_file(${T_FILE} ${TEST_NETWORK_HEADER_FILE})
    message(STATUS "Generated ${TEST_NETWORK_HEADER_FILE}")
endforeach()


# Add the generated header files to list of includes
include_directories(${PROJECT_BINARY_DIR}/include)
message("")

add_subdirectory(dynamics)
add_subdirectory(cost_functions)
add_subdirectory(feedback_controllers)
add_subdirectory(mppi_core)
add_subdirectory(controllers)
add_subdirectory(misc)
add_subdirectory(sampling_distributions)
add_subdirectory(shaping_functions)
add_subdirectory(math_utils)
add_subdirectory(integration)
add_subdirectory(texture_helpers)
add_subdirectory(nn_helpers)
