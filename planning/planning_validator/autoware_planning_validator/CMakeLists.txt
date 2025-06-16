cmake_minimum_required(VERSION 3.22)
project(autoware_planning_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()

# planning validator
ament_auto_add_library(autoware_planning_validator_component SHARED
  include/autoware/planning_validator/debug_marker.hpp
  include/autoware/planning_validator/utils.hpp
  include/autoware/planning_validator/types.hpp
  include/autoware/planning_validator/plugin_interface.hpp
  include/autoware/planning_validator/manager.hpp
  include/autoware/planning_validator/node.hpp
  src/manager.cpp
  src/node.cpp
  src/debug_marker.cpp
  src/utils.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::planning_validator::PlanningValidatorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# invalid trajectory publisher (for debug)
ament_auto_add_library(invalid_trajectory_publisher_node SHARED
  src/invalid_trajectory_publisher/invalid_trajectory_publisher.cpp
)
rclcpp_components_register_node(invalid_trajectory_publisher_node
  PLUGIN "autoware::planning_validator::InvalidTrajectoryPublisherNode"
  EXECUTABLE invalid_trajectory_publisher
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/PlanningValidatorStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# to use a message defined in the same package
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(${PROJECT_NAME}_component
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${PROJECT_NAME}_component "${cpp_typesupport_target}")
endif()

if(BUILD_TESTING)
  find_package(autoware_planning_validator_test_utils REQUIRED)
  ament_add_ros_isolated_gtest(test_autoware_planning_validator
    test/src/test_main.cpp
    test/src/test_planning_validator_node_interface.cpp
  )
  ament_target_dependencies(test_autoware_planning_validator
    rclcpp
    autoware_planning_msgs
    autoware_planning_validator_test_utils
  )
  target_link_libraries(test_autoware_planning_validator
    ${PROJECT_NAME}_component
    ${autoware_planning_validator_test_utils_LIBRARIES}
  )
  set_tests_properties(test_autoware_planning_validator PROPERTIES TIMEOUT 120)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
