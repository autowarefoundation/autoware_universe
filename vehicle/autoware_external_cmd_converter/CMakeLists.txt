cmake_minimum_required(VERSION 3.14)
project(autoware_external_cmd_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_external_cmd_converter SHARED
  src/node.cpp
)

autoware_agnocast_wrapper_register_node(autoware_external_cmd_converter
  PLUGIN "autoware::external_cmd_converter::ExternalCmdConverterNode"
  EXECUTABLE external_cmd_converter_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING AND NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    tests/test_external_cmd_converter.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    autoware_external_cmd_converter
  )
endif()


ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
