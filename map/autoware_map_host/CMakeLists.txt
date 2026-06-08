cmake_minimum_required(VERSION 3.14)
project(autoware_map_host)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_map_host plugins.xml)

find_package(autoware_map_loader REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

ament_auto_add_library(autoware_map_loader_host_component SHARED
  src/map_loader_host/map_loader_host.cpp
)

rclcpp_components_register_node(autoware_map_loader_host_component
  PLUGIN "autoware::map_loader::MapLoaderHost"
  EXECUTABLE autoware_map_loader_host_node
)

ament_auto_add_library(autoware_map_loader_host_plugins SHARED
  src/map_loader_host/plugins/map_projection_loader_plugin.cpp
  src/map_loader_host/plugins/pointcloud_map_loader_plugin.cpp
  src/map_loader_host/plugins/lanelet2_map_loader_plugin.cpp
  src/map_loader_host/plugins/lanelet2_visualization_plugin.cpp
  src/map_loader_host/plugins/vector_map_tf_generator_plugin.cpp
  src/map_loader_host/plugins/map_hash_generator_plugin.cpp
)

# Vendored pointcloud module headers (include/) are compile-time only.
# Implementations are linked from autoware_map_loader libraries.
target_include_directories(autoware_map_loader_host_plugins PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(autoware_map_loader_host_plugins
  ${autoware_map_loader_LIBRARIES}
  OpenSSL::Crypto
)
ament_target_dependencies(autoware_map_loader_host_plugins
  autoware_map_loader
  autoware_utils
  tier4_external_api_msgs
)

autoware_ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
