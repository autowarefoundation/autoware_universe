cmake_minimum_required(VERSION 3.14)
project(autoware_feature_environment_recognizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FeatureEnvironment.msg"
  DEPENDENCIES
  std_msgs
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/feature_environment_recognizer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link message typesupport
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(${PROJECT_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::feature_environment_recognizer::FeatureEnvironmentRecognizer"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTATOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
