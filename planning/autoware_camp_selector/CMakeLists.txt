cmake_minimum_required(VERSION 3.16)
project(autoware_camp_selector VERSION 0.1.0 LANGUAGES C CXX)

option(CAMP_STANDALONE "Build the same library/tests without a ROS underlay" OFF)
option(CAMP_BUILD_TENSOR_MATERIALIZER "Build the preserved tensor/map atom adapter" ON)
if(NOT CAMP_STANDALONE)
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(autoware_internal_planning_msgs REQUIRED)
  find_package(autoware_planning_msgs REQUIRED)
  find_package(autoware_vehicle_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
endif()
find_package(nlohmann_json REQUIRED)

add_library(camp_selector_lib src/camp_ranker.cpp)
add_library(autoware::camp_selector ALIAS camp_selector_lib)
set_target_properties(camp_selector_lib PROPERTIES EXPORT_NAME camp_selector)
target_compile_features(camp_selector_lib PUBLIC cxx_std_17)
target_include_directories(camp_selector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camp_selector_lib PRIVATE nlohmann_json::nlohmann_json)
if(CAMP_BUILD_TENSOR_MATERIALIZER)
  find_package(Eigen3 REQUIRED)
  find_package(Boost REQUIRED)
  target_sources(camp_selector_lib PRIVATE src/camp_atom_materializer.cpp)
  target_link_libraries(camp_selector_lib PUBLIC Eigen3::Eigen PRIVATE Boost::boost)
endif()

if(NOT CAMP_STANDALONE)
  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/CampCandidatePool.msg
    msg/CampSelection.msg
    DEPENDENCIES std_msgs autoware_internal_planning_msgs)
  add_executable(camp_selector_node src/camp_selector_node.cpp)
  target_compile_features(camp_selector_node PRIVATE cxx_std_17)
  target_link_libraries(camp_selector_node PRIVATE camp_selector_lib)
  ament_target_dependencies(camp_selector_node PUBLIC rclcpp rcl_interfaces autoware_internal_planning_msgs
    autoware_planning_msgs autoware_vehicle_msgs std_msgs)
  rosidl_get_typesupport_target(camp_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(camp_selector_node PRIVATE ${camp_typesupport})
  install(TARGETS camp_selector_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
endif()

include(CTest)
if(BUILD_TESTING)
  set(camp_test_sources test/test_camp_ranker.cpp test/test_adapter_contracts.cpp)
  if(CAMP_BUILD_TENSOR_MATERIALIZER)
    list(APPEND camp_test_sources test/camp_atom_materializer_test.cpp test/lane_boundary_test.cpp)
  endif()
  if(CAMP_STANDALONE)
    find_package(GTest REQUIRED)
    add_executable(test_camp_selector ${camp_test_sources})
    target_link_libraries(test_camp_selector GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(test_camp_selector)
  else()
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_camp_selector ${camp_test_sources})
  endif()
  target_link_libraries(test_camp_selector camp_selector_lib)
  target_compile_definitions(test_camp_selector PRIVATE
    CAMP_TEST_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/camp_v26_k8_50k.json")
  if(NOT CAMP_STANDALONE)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_camp_selector_ros test/test_ros_selector.py
      TIMEOUT 60
      ENV "CAMP_SELECTOR_EXECUTABLE=$<TARGET_FILE:camp_selector_node>")
  endif()
endif()

install(TARGETS camp_selector_lib EXPORT camp_selector_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md package.xml DESTINATION share/${PROJECT_NAME})
if(CAMP_STANDALONE)
  include(CMakePackageConfigHelpers)
  configure_package_config_file(camp_selectorConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME})
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    DESTINATION lib/cmake/${PROJECT_NAME})
  install(EXPORT camp_selector_targets NAMESPACE autoware::
    DESTINATION lib/cmake/${PROJECT_NAME})
else()
  ament_export_targets(camp_selector_targets HAS_LIBRARY_TARGET NAMESPACE autoware::)
  ament_export_dependencies(nlohmann_json rosidl_default_runtime)
  if(CAMP_BUILD_TENSOR_MATERIALIZER)
    ament_export_dependencies(Eigen3 Boost)
  endif()
  ament_package()
endif()
