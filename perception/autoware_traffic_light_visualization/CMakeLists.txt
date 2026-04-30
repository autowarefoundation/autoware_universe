cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_visualization)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)

ament_auto_add_library(traffic_light_roi_visualizer SHARED
  src/traffic_light_roi_visualizer/node.cpp
  src/traffic_light_roi_visualizer/shape_draw.cpp
)

target_include_directories(traffic_light_roi_visualizer SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(traffic_light_roi_visualizer
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(traffic_light_roi_visualizer
  PLUGIN "autoware::traffic_light::TrafficLightRoiVisualizerNode"
  EXECUTABLE traffic_light_visualization_node
)

ament_auto_add_library(traffic_light_map_visualizer SHARED
  src/traffic_light_map_visualizer/node.cpp
)

rclcpp_components_register_node(traffic_light_map_visualizer
  PLUGIN "autoware::traffic_light::TrafficLightMapVisualizerNode"
  EXECUTABLE traffic_light_map_visualizer_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_traffic_light_map_visualizer
    test/test_traffic_light_map_visualizer.cpp
  )
  target_include_directories(test_traffic_light_map_visualizer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_traffic_light_map_visualizer
    traffic_light_map_visualizer
  )
  ament_target_dependencies(test_traffic_light_map_visualizer
    autoware_lanelet2_extension
    autoware_lanelet2_utils
    autoware_map_msgs
    autoware_perception_msgs
    rclcpp
    visualization_msgs
  )
endif()

# Copy the assets directory to the installation directory
install(
    DIRECTORY images/
    DESTINATION share/${PROJECT_NAME}/images
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
