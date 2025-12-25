cmake_minimum_required(VERSION 3.16.3) # Ubuntu 20.04 default CMake version

project(autoware_scenario_simulator_v2_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(pluginlib REQUIRED)

ament_auto_find_build_dependencies()


ament_auto_add_library(${PROJECT_NAME} SHARED
  src/converter_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::scenario_simulator_v2_adapter::MetricConverter"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_scenario_simulator_v2_adapter
    test/test_converter_node.cpp
  )
  target_link_libraries(test_scenario_simulator_v2_adapter
    ${PROJECT_NAME}
  )
endif()

ament_auto_package()
