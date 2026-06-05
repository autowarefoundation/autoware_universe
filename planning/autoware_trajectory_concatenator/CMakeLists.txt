cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_concatenator)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/detail/trajectory_concatenator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_auto_find_test_dependencies()

  ament_add_gtest(test_trajectory_concatenator
    test/test_trajectory_concatenator.cpp
  )
  target_link_libraries(test_trajectory_concatenator
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_trajectory_concatenator_wrapper
    test/test_trajectory_concatenator_wrapper.cpp
  )
  target_link_libraries(test_trajectory_concatenator_wrapper
    ${PROJECT_NAME}
  )
endif()

ament_auto_package()
