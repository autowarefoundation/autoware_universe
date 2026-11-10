cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_follower_node)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(CONTROLLER_NODE controller_node)
ament_auto_add_library(${CONTROLLER_NODE} SHARED
  include/autoware/trajectory_follower_node/controller_node.hpp
  include/autoware/trajectory_follower_node/trajectory_follower.hpp
  src/controller_node.cpp
  src/trajectory_follower.cpp
)

autoware_agnocast_wrapper_register_node(${CONTROLLER_NODE}
  PLUGIN "autoware::motion::control::trajectory_follower_node::Controller"
  EXECUTABLE ${CONTROLLER_NODE}_exe
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

# simple trajectory follower
set(SIMPLE_TRAJECTORY_FOLLOWER_NODE simple_trajectory_follower)
ament_auto_add_library(${SIMPLE_TRAJECTORY_FOLLOWER_NODE} SHARED
  include/autoware/trajectory_follower_node/simple_trajectory_follower.hpp
  src/simple_trajectory_follower.cpp
)

autoware_agnocast_wrapper_register_node(${SIMPLE_TRAJECTORY_FOLLOWER_NODE}
  PLUGIN "simple_trajectory_follower::SimpleTrajectoryFollower"
  EXECUTABLE ${SIMPLE_TRAJECTORY_FOLLOWER_NODE}_exe
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  set(TRAJECTORY_FOLLOWER_NODES_TEST test_trajectory_follower_node)
  ament_add_ros_isolated_gtest(${TRAJECTORY_FOLLOWER_NODES_TEST}
    test/test_controller_node.cpp
  )
  ament_target_dependencies(${TRAJECTORY_FOLLOWER_NODES_TEST} autoware_fake_test_node ament_index_cpp)
  target_link_libraries(
    ${TRAJECTORY_FOLLOWER_NODES_TEST} ${CONTROLLER_NODE})

  ament_add_gtest(test_trajectory_follower
    test/test_trajectory_follower.cpp
  )
  target_link_libraries(test_trajectory_follower ${CONTROLLER_NODE})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  param
  launch
  config
)
