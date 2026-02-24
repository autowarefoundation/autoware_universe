cmake_minimum_required(VERSION 3.14)
project(autoware_detected_object_feature_remover)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)

set(DETECTED_OBJECT_FEATURE_REMOVER_DEPENDENCIES
  PCL
  OpenCV
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  lib/convert.cpp
  src/detected_object_feature_remover_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${DETECTED_OBJECT_FEATURE_REMOVER_DEPENDENCIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::detected_object_feature_remover::DetectedObjectFeatureRemover"
  EXECUTABLE detected_object_feature_remover_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_detected_object_feature_remover_node.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${${PROJECT_NAME}_LIBRARIES})
  ament_target_dependencies(test_${PROJECT_NAME}
    ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
    ${${PROJECT_NAME}_FOUND_TEST_DEPENDS}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
