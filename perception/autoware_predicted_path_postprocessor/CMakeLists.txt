cmake_minimum_required(VERSION 3.14)
project(autoware_predicted_path_postprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- find dependencies ---
find_package(autoware_cmake REQUIRED)
autoware_package()

file(GLOB_RECURSE SOURCE_FILES src/*.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

rclcpp_components_register_node(
    ${PROJECT_NAME} PLUGIN
    "autoware::predicted_path_postprocessor::PredictedPathPostprocessorNode"
    EXECUTABLE ${PROJECT_NAME}_exe)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
