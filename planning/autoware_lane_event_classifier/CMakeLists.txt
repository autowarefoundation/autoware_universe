cmake_minimum_required(VERSION 3.14)
project(autoware_lane_event_classifier)

find_package(autoware_cmake REQUIRED)
autoware_package()

unset(PYTHON_EXECUTABLE CACHE)
unset(PYTHON_EXECUTABLE)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DrivingState.msg"
  "msg/DrivingFactor.msg"
  DEPENDENCIES std_msgs
)

generate_parameter_library(lane_event_classifier_parameters
  schema/lane_event_classifier.schema.yaml
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/lane_event_classifier_node.cpp
  src/debug.cpp
  src/lane_following/checker.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  lane_event_classifier_parameters
)

# Suppress deprecated-declarations from generate_parameter_library internals
target_compile_options(${PROJECT_NAME}_lib PRIVATE -Wno-deprecated-declarations)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::lane_event_classifier::LaneEventClassifierNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Per-classifier gtests (lane_following, lane_change, lane_crossing) are added alongside each
  # classifier's real logic in its own follow-up PR.
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  param
  schema
)
