# Double integrator dynamics tests
set(TARGET_NAME miscellaneous_tests)

add_executable(${TARGET_NAME} ${PROJECT_SOURCE_DIR}/tests/test_main.cpp miscellaneous_tests.cu)
target_link_libraries(${TARGET_NAME} gtest gmock gtest_main double_integrator_mppi)
set_target_properties(${TARGET_NAME} PROPERTIES FOLDER test)

gtest_add_tests(TARGET miscellaneous_tests)

