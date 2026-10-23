cmake_minimum_required(VERSION 3.14)
project(autoware_pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

# autoware_pure_pursuit_core
ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/${PROJECT_NAME}_core/planning_utils.cpp
  src/${PROJECT_NAME}_core/${PROJECT_NAME}.cpp
)

# autoware_pure_pursuit
ament_auto_add_library(${PROJECT_NAME}_lateral_controller SHARED
  src/${PROJECT_NAME}/${PROJECT_NAME}_lateral_controller.cpp
  src/${PROJECT_NAME}/${PROJECT_NAME}_lateral_controller_node.cpp
  src/${PROJECT_NAME}/ros_log_writer.cpp
)

target_link_libraries(${PROJECT_NAME}_lateral_controller
  ${PROJECT_NAME}_core
)

rclcpp_components_register_node(${PROJECT_NAME}_lateral_controller
  PLUGIN "autoware::pure_pursuit::PurePursuitLateralControllerNode"
  EXECUTABLE ${PROJECT_NAME}_lateral_controller_exe
)

if(BUILD_TESTING)
  set(TEST_LATERAL_CONTROLLER_EXE test_pure_pursuit_lateral_controller)
  ament_add_ros_isolated_gtest(${TEST_LATERAL_CONTROLLER_EXE}
    test/test_pure_pursuit_lateral_controller.cpp
  )
  target_link_libraries(${TEST_LATERAL_CONTROLLER_EXE} ${PROJECT_NAME}_lateral_controller)
  ament_target_dependencies(${TEST_LATERAL_CONTROLLER_EXE} ament_index_cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
