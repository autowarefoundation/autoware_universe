cmake_minimum_required(VERSION 3.14)
project(autoware_mppi_optimizer LANGUAGES C CXX CUDA)

set(CMAKE_CUDA_ARCHITECTURES OFF)

find_package(autoware_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(MPPI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MPPI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

include(MPPIGenericToolsConfig)

add_subdirectory(lib/MPPI-Generic)

add_library(mppi_extensions INTERFACE)
add_library(MppiExtensions::extensions ALIAS mppi_extensions)
target_include_directories(mppi_extensions BEFORE INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(mppi_extensions INTERFACE MPPI::MPPI)

find_package(Eigen3 REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(first_order_dubins_mppi SHARED
  src/first_order_dubins/first_order_dubins_mppi_interface.cu
)
target_include_directories(first_order_dubins_mppi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(first_order_dubins_mppi PUBLIC
  mppi_extensions
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
  autoware_perception_msgs::autoware_perception_msgs__rosidl_generator_cpp
  autoware_planning_msgs::autoware_planning_msgs__rosidl_generator_cpp
  nav_msgs::nav_msgs__rosidl_generator_cpp
)
set_target_properties(first_order_dubins_mppi PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
  CUDA_STANDARD 17
)

autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mppi_optimizer.cpp
)
target_link_libraries(${PROJECT_NAME} first_order_dubins_mppi)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::mppi_optimizer::MppiOptimizer"
  EXECUTABLE autoware_mppi_optimizer_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS first_order_dubins_mppi cnpy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(first_order_dubins_mppi)

ament_auto_package(INSTALL_TO_SHARE config launch)
