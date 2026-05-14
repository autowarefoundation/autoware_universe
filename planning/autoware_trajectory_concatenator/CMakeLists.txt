cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_concatenator)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/detail/trajectory_concatenator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

ament_auto_package()
