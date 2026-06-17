set(GTEST_LIBRARIES gtest gmock gtest_main)
file(GLOB TARGET_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cu)

foreach(T_FILE IN LISTS TARGET_SRCS)
    get_filename_component(T_NAME ${T_FILE} NAME_WE)
    add_executable(${T_NAME} ${PROJECT_SOURCE_DIR}/tests/test_main.cpp ${T_FILE})
    target_link_libraries(${T_NAME}
            ${GTEST_LIBRARIES}
            ${MPPI_HEADER_LIBRARY_NAME})
    gtest_add_tests(TARGET ${T_NAME})
    set_target_properties(${T_NAME} PROPERTIES FOLDER test)
endforeach()
