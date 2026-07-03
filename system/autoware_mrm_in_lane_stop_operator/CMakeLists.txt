cmake_minimum_required(VERSION 3.14)
project(autoware_mrm_in_lane_stop_operator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/mrm_in_lane_stop_operator.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::mrm_in_lane_stop_operator::MrmInLaneStopOperator"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
