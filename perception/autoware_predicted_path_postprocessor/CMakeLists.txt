cmake_minimum_required(VERSION 3.14)
project(autoware_predicted_path_postprocessor)

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
    find_package(ament_cmake_ros REQUIRED)
    ament_auto_find_test_dependencies()

    ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
        test/test_collision.cpp
        test/test_refine_by_speed.cpp
        test/test_refine_penetration_by_static_objects.cpp)

    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
    )
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
