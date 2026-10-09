cmake_minimum_required(VERSION 3.14)
project(autoware_mpc_lateral_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(fmt REQUIRED)

set(MPC_LAT_CON_LIB ${PROJECT_NAME}_lib)
ament_auto_add_library(${MPC_LAT_CON_LIB} SHARED
  src/mpc_lateral_controller.cpp
  src/ros_log_writer.cpp
  src/mpc_lateral_controller_node.cpp
  src/lowpass_filter.cpp
  src/steering_predictor.cpp
  src/mpc.cpp
  src/mpc_trajectory.cpp
  src/mpc_utils.cpp
  src/qp_solver/qp_solver_osqp.cpp
  src/qp_solver/qp_solver_unconstraint_fast.cpp
  src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/vehicle_model/vehicle_model_interface.cpp
)

if(BUILD_TESTING)
  set(TEST_LAT_SOURCES
    test/test_mpc.cpp
    test/test_mpc_lateral_controller.cpp
    test/test_mpc_utils.cpp
    test/test_steering_predictor.cpp
    test/test_lowpass_filter.cpp
  )
  set(TEST_LATERAL_CONTROLLER_EXE test_lateral_controller)
  # Several tests in test_mpc_lateral_controller.cpp run the controller for up to 69 cycles.
  # A coverage build makes one cycle about 27 times slower, from 6 ms to 160 ms, so the whole
  # executable runs for around 75 seconds and passes the sixty second default.
  ament_add_ros_isolated_gtest(${TEST_LATERAL_CONTROLLER_EXE} ${TEST_LAT_SOURCES} TIMEOUT 300)
  target_link_libraries(${TEST_LATERAL_CONTROLLER_EXE} ${MPC_LAT_CON_LIB})
  ament_target_dependencies(${TEST_LATERAL_CONTROLLER_EXE} ament_index_cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE
  param
)
