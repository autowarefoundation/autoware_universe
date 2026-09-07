cmake_minimum_required(VERSION 3.14)
project(autoware_in_lane_mrm_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

generate_parameter_library(in_lane_mrm_planner_parameters
  param/in_lane_mrm_planner_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/in_lane_mrm_planner_node.cpp
  src/in_lane_mrm_trajectory_planner.cpp
  src/in_lane_mrm_trajectory_modifier.cpp
  src/trajectory_smoother.cpp
  src/path_planner.cpp
  src/obstacle_stop_utils.cpp
  src/mrm_obstacle_stop_planner.cpp
  src/mrm_stop_velocity_planner.cpp
  src/in_lane_mrm_trajectory_validator.cpp
  src/trajectory_latcher.cpp
  src/predicted_objects_latcher.cpp
  src/mrm_trigger_relay_node.cpp
  src/mrm_in_lane_stop_auxiliary_commands_node.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  in_lane_mrm_planner_parameters
  ${PCL_LIBRARIES}
)

target_compile_options(${PROJECT_NAME}_lib PRIVATE
  -Wno-deprecated-declarations
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::in_lane_mrm_planner::InLaneMrmPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::in_lane_mrm_planner::MrmTriggerRelayNode"
  EXECUTABLE mrm_trigger_relay_node
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::in_lane_mrm_planner::MrmInLaneStopAuxiliaryCommandsNode"
  EXECUTABLE mrm_in_lane_stop_auxiliary_commands_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
    test/test_lateral_offset.cpp
    test/test_mrm_stop_velocity_planner.cpp
    test/test_in_lane_mrm_trajectory_validator.cpp
    test/test_trajectory_latcher.cpp
    test/test_param_validation.cpp
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_lib
  )

  target_compile_options(test_${PROJECT_NAME} PRIVATE
    -Wno-error=deprecated-declarations
  )
endif()

install(
  PROGRAMS scripts/in_lane_mrm_rosbag_common.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

install(
  PROGRAMS scripts/in_lane_mrm_planner_rosbag_analyzer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME in_lane_mrm_planner_rosbag_analyzer
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

install(
  PROGRAMS scripts/in_lane_mrm_planner_rosbag_record.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME in_lane_mrm_planner_rosbag_record
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
    schema
)
