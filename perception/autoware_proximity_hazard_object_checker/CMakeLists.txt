cmake_minimum_required(VERSION 3.14)
project(autoware_proximity_hazard_object_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/proximity_hazard_object_checker.cpp
  src/proximity_hazard_object_checker_node.cpp
)

# Clear stale Python interpreter state so rosidl can resolve it correctly.
unset(PYTHON_EXECUTABLE CACHE)
unset(PYTHON_EXECUTABLE)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ProximityHazardObject.msg"
  "msg/ProximityHazardObjects.msg"
  DEPENDENCIES
  builtin_interfaces
  unique_identifier_msgs
  autoware_perception_msgs
)

# to use messages defined in the same package from C++
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(${PROJECT_NAME}_component
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${PROJECT_NAME}_component "${cpp_typesupport_target}")
endif()

target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_param
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::proximity_hazard_object_checker::ProximityHazardObjectCheckerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
