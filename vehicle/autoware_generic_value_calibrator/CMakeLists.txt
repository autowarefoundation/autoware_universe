cmake_minimum_required(VERSION 3.14)
project(autoware_generic_value_calibrator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/generic_value_calibrator_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

# Explicitly link against TF2 libraries
target_link_libraries(${PROJECT_NAME}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::generic_value_calibrator::GenericValueCalibrator"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_calibrator_utils
    test/test_calibrator_utils.cpp
  )
  if(TARGET test_calibrator_utils)
    target_link_libraries(test_calibrator_utils ${PROJECT_NAME})
    ament_target_dependencies(test_calibrator_utils
      ament_index_cpp
      rclcpp
    )
  endif()

  # Python tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_calc_utils
    test/test_calc_utils.py
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
    APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}/scripts"
  )
endif()

install(PROGRAMS
  scripts/topic_converter.py
  scripts/generic_value_map_server.py
  scripts/calc_utils.py
  scripts/csv_reader.py
  scripts/plotter.py
  scripts/config.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    data
    launch
    rviz
    schema
)
