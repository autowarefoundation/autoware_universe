cmake_minimum_required(VERSION 3.8)
project(distance_selective_rsu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(distance_selective_rsu_node 
  src/distance_selective_rsu_node.cpp
)

ament_target_dependencies(distance_selective_rsu_node
  rclcpp
  autoware_perception_msgs
  tf2_ros
  message_filters
  geometry_msgs
)

install(TARGETS
  distance_selective_rsu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
