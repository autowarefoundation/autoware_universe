cmake_minimum_required(VERSION 3.5)
project(autoware_deviation_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(deviation_estimator_lib SHARED
  src/deviation_estimator.cpp
  src/utils.cpp
  src/gyro_bias_module.cpp
  src/velocity_coef_module.cpp
  src/logger.cpp
  src/validation_module.cpp
)

# as a ros2 node
ament_auto_add_executable(deviation_estimator src/deviation_estimator_node.cpp)
target_compile_options(deviation_estimator PUBLIC -g -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(deviation_estimator deviation_estimator_lib)
target_include_directories(deviation_estimator PUBLIC include)

# as a static tool
ament_auto_add_executable(deviation_estimator_unit_tool src/deviation_estimator_main.cpp)
target_compile_options(deviation_estimator_unit_tool PUBLIC -g -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(deviation_estimator_unit_tool deviation_estimator_lib)
target_include_directories(deviation_estimator_unit_tool PUBLIC include)

if(BUILD_TESTING)
  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})

    ament_add_gtest(${test_name} ${filepath})
    target_link_libraries("${test_name}" deviation_estimator_lib)
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()

  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_FILES
    test/test_gyro_stddev.cpp
    test/test_gyro_bias.cpp
    test/test_utils.cpp)

  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()


ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
