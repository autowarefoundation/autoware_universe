cmake_minimum_required(VERSION 3.5)
project(autoware_crosswalk_traffic_light_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

###########
## Build ##
###########

include_directories()

ament_auto_add_library(autoware_crosswalk_traffic_light_estimator SHARED
  src/node.cpp
)

rclcpp_components_register_node(autoware_crosswalk_traffic_light_estimator
  PLUGIN "autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorNode"
  EXECUTABLE crosswalk_traffic_light_estimator_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_crosswalk_traffic_light_estimator_node
    test/test_crosswalk_traffic_light_estimator_node.cpp
  )
  target_link_libraries(test_crosswalk_traffic_light_estimator_node
    autoware_crosswalk_traffic_light_estimator
  )
endif()

#############
## Install ##
#############

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        config
)
