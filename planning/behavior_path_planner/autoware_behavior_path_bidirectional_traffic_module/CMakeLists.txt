cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_bidirectional_traffic_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_path_planner plugins.xml)

# glob cpp
file(GLOB_RECURSE CPP_SOURCES src/*.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED ${CPP_SOURCES})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
    ament_add_ros_isolated_gtest(test_${PROJECT_NAME} ${TEST_SOURCES})
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE config test_data test_map)
