cmake_minimum_required(VERSION 3.14)
project(autoware_candidate_bank)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/candidate/trajectory_candidate_manager.cpp
  src/internal_debug/internal_debug_logger.cpp
  src/preprocessing/lane_segments.cpp
  src/preprocessing/traffic_signals.cpp
  src/conversion/lanelet.cpp
  src/rule_context.cpp
)
target_include_directories(${PROJECT_NAME}_core SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)

ament_auto_add_executable(candidate_bank_node
  src/candidate_bank_node.cpp
)
target_link_libraries(candidate_bank_node ${PROJECT_NAME}_core nlohmann_json::nlohmann_json)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_bank.cpp
    test/test_exact_pair_queue.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}_core)
endif()

install(PROGRAMS
  scripts/analyze_candidate_bank_internal_log.py
  scripts/analyze_static_clearance.py
  scripts/inspect_graph.py
  scripts/plot_candidate_cross_collision_matrix.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
