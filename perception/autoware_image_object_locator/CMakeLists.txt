cmake_minimum_required(VERSION 3.14)
project(autoware_image_object_locator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)

# Initialize ament_auto
ament_auto_find_build_dependencies()
autoware_package()

set(${PROJECT_NAME}_src
  src/bbox_object_locator_node.cpp
)

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_src}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::image_object_locator::BboxObjectLocatorNode"
  EXECUTABLE bbox_object_locator_node
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate package
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
