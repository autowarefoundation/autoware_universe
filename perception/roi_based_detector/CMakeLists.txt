cmake_minimum_required(VERSION 3.8)
project(roi_based_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# Initialize ament_auto
ament_auto_find_build_dependencies()
autoware_package()

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/roi_based_detector_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "roi_based_detector::RoiBasedDetectorNode"
  EXECUTABLE roi_based_detector_node
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
