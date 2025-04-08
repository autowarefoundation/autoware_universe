cmake_minimum_required(VERSION 3.14)
project(autoware_mission_planner_universe)

# TODO(youtalk): workaround to avoid 'missing tinyxml2::tinyxml2'
# See also https://github.com/autowarefoundation/autoware_universe/blob/main/visualization/tier4_traffic_light_rviz_plugin/CMakeLists.txt#L5-L20
find_package(TinyXML2 CONFIG QUIET)
if(NOT TinyXML2_FOUND)
  find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
  find_library(TINYXML2_LIBRARY tinyxml2)
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(TinyXML2 DEFAULT_MSG TINYXML2_LIBRARY TINYXML2_INCLUDE_DIR)
  mark_as_advanced(TINYXML2_INCLUDE_DIR TINYXML2_LIBRARY)
  if(NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY IMPORTED_LOCATION ${TINYXML2_LIBRARY})
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${TINYXML2_INCLUDE_DIR})
    list(APPEND TinyXML2_TARGETS tinyxml2::tinyxml2)
  endif()
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(goal_pose_visualizer_component SHARED
  src/goal_pose_visualizer/goal_pose_visualizer.cpp
)

rclcpp_components_register_node(goal_pose_visualizer_component
  PLUGIN "autoware::mission_planner_universe::GoalPoseVisualizer"
  EXECUTABLE goal_pose_visualizer
)

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/mission_planner/arrival_checker.cpp
  src/mission_planner/service_utils.cpp
  src/mission_planner/mission_planner.cpp
  src/mission_planner/route_selector.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::mission_planner_universe::MissionPlanner"
  EXECUTABLE mission_planner
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::mission_planner_universe::RouteSelector"
  EXECUTABLE route_selector
)

ament_auto_add_library(${PROJECT_NAME}_lanelet2_plugins SHARED
  src/lanelet2_plugins/default_planner.cpp
  src/lanelet2_plugins/utility_functions.cpp
)
pluginlib_export_plugin_description_file(autoware_mission_planner_universe plugins/plugin_description.xml)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
  test/test_lanelet2_plugins_default_planner.cpp
  test/test_utility_functions.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_lanelet2_plugins
  )
  ament_target_dependencies(test_${PROJECT_NAME}
    autoware_test_utils
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
