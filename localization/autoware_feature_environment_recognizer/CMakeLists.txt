cmake_minimum_required(VERSION 3.14)
project(autoware_feature_environment_recognizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Clear ${PYTHON_EXECUTABLE} defined by mrt_cmake_module so that rosidl_generate_interfaces can look for it properly
unset(PYTHON_EXECUTABLE)

# Find Python3 explicitly for rosidl_generate_interfaces
find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FeatureEnvironment.msg"
  DEPENDENCIES
  std_msgs
)

# Create library with different name to avoid conflict with rosidl_generate_interfaces target
set(LIBRARY_NAME ${PROJECT_NAME}_lib)
ament_auto_add_library(${LIBRARY_NAME} SHARED
  src/feature_environment_recognizer.cpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# Boost geometry is header-only, no linking needed
find_package(Boost REQUIRED)

# Link message typesupport
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(${LIBRARY_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${LIBRARY_NAME} "${cpp_typesupport_target}")
endif()

rclcpp_components_register_node(${LIBRARY_NAME}
  PLUGIN "autoware::feature_environment_recognizer::FeatureEnvironmentRecognizer"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
