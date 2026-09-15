cmake_minimum_required(VERSION 3.14)
project(autoware_redundancy_adapi_switcher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/redundancy_adapi_switcher/redundancy_adapi_switcher.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::redundancy_adapi_switcher::RedundancyAdapiSwitcher"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE launch)
