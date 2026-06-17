add_executable(generic_feedback_controller_test ${PROJECT_SOURCE_DIR}/tests/test_main.cpp generic_feedback_controller_test.cu)
target_link_libraries(generic_feedback_controller_test gtest
        gmock
        gtest_main
        cartpole_mppi)
gtest_discover_tests(generic_feedback_controller_test)
set_target_properties(generic_feedback_controller_test PROPERTIES FOLDER test)

add_executable(ddp_tests ${PROJECT_SOURCE_DIR}/tests/test_main.cpp ddp_test.cu)
target_link_libraries(ddp_tests gtest
        gmock
        gtest_main
        cartpole_mppi)
gtest_discover_tests(ddp_tests)
set_target_properties(ddp_tests PROPERTIES FOLDER test)
