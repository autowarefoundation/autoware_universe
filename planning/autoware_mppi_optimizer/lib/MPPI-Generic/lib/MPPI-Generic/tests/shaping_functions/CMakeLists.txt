# Rollout kernel test suite
set(SHAPING_FUNCTION_TARGET_NAME shaping_function_tests)

add_executable(${SHAPING_FUNCTION_TARGET_NAME} ${PROJECT_SOURCE_DIR}/tests/test_main.cpp shaping_function_test.cu)
target_link_libraries(${SHAPING_FUNCTION_TARGET_NAME} gtest
        gmock
        gtest_main
        ${MPPI_HEADER_LIBRARY_NAME})

gtest_discover_tests(${SHAPING_FUNCTION_TARGET_NAME})
set_target_properties(${SHAPING_FUNCTION_TARGET_NAME} PROPERTIES FOLDER test)

# Rollout kernel test suite
set(CEM_SHAPING_FUNCTION_TARGET_NAME cem_shaping_function_tests)

add_executable(${CEM_SHAPING_FUNCTION_TARGET_NAME} ${PROJECT_SOURCE_DIR}/tests/test_main.cpp cem_shaping_function_test.cu)
target_link_libraries(${CEM_SHAPING_FUNCTION_TARGET_NAME} gtest
        gmock
        gtest_main
        ${MPPI_HEADER_LIBRARY_NAME})

gtest_discover_tests(${CEM_SHAPING_FUNCTION_TARGET_NAME})
set_target_properties(${CEM_SHAPING_FUNCTION_TARGET_NAME} PROPERTIES FOLDER test)
