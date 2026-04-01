# Unit Test Configuration for autoware_tensorrt_vad
# Add this to your existing CMakeLists.txt in the testing section

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Focused unit test for JSON param loading and version checking
  ament_add_gtest(test_param_loader
    test_param_loader.cpp
  )

  target_include_directories(test_param_loader PRIVATE
    ${PROJECT_SOURCE_DIR}/src
  )

  install(TARGETS
    test_param_loader
    DESTINATION lib/${PROJECT_NAME}
  )

  # Pure helper tests for classification loader
  ament_add_gtest(test_classification_loader
    test_classification_loader.cpp
  )

  target_include_directories(test_classification_loader PRIVATE
    ${PROJECT_SOURCE_DIR}/src
  )

  install(TARGETS
    test_classification_loader
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

# To run the tests:
# colcon build --packages-select autoware_tensorrt_vad
# colcon test --packages-select autoware_tensorrt_vad
# colcon test-result --verbose
