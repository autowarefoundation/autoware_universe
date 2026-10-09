cmake_minimum_required(VERSION 3.16)
project(autoware_camp_diffusion_adapter LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
set(adapter_dependencies
  autoware_camp_selector
  autoware_diffusion_planner
  autoware_internal_planning_msgs
  autoware_lanelet2_utils
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_factor_interface
  autoware_planning_msgs
  autoware_utils_uuid
  autoware_vehicle_info_utils
  autoware_vehicle_msgs
  geometry_msgs
  lanelet2_core
  nav_msgs
  rcl_interfaces
  rclcpp
  std_msgs
  std_srvs
  unique_identifier_msgs)
foreach(dependency IN LISTS adapter_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_executable(camp_diffusion_adapter_node
  src/adapter.cpp
  src/camp_diffusion_adapter_node.cpp)
target_compile_features(camp_diffusion_adapter_node PRIVATE cxx_std_17)
target_include_directories(camp_diffusion_adapter_node PRIVATE include)
ament_target_dependencies(camp_diffusion_adapter_node PUBLIC ${adapter_dependencies})
target_link_libraries(camp_diffusion_adapter_node PRIVATE autoware::camp_selector)

install(TARGETS camp_diffusion_adapter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
