cmake_minimum_required(VERSION 3.14)
project(autoware_detected_object_feature_remover)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/detected_object_feature_remover_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::detected_object_feature_remover::DetectedObjectFeatureRemover"
  EXECUTABLE detected_object_feature_remover_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  # run node unit testings with ROS_DOMAIN_ID isolation
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_node
    test/test_detected_object_feature_remover_node.cpp
  )
  target_include_directories(test_${PROJECT_NAME}_node PRIVATE src)
  target_link_libraries(test_${PROJECT_NAME}_node ${${PROJECT_NAME}_LIBRARIES})
  ament_target_dependencies(test_${PROJECT_NAME}_node
    ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
    ${${PROJECT_NAME}_FOUND_TEST_DEPENDS}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
