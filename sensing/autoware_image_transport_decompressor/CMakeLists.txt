cmake_minimum_required(VERSION 3.14)
project(autoware_image_transport_decompressor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/image_transport_decompressor.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

ament_auto_add_library(${PROJECT_NAME}_ros SHARED
  src/image_transport_decompressor_node.cpp
)

target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}_ros
  PLUGIN "autoware::image_preprocessor::ImageTransportDecompressor"
  EXECUTABLE image_transport_decompressor_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_gtest(test_image_transport_decompressor
    test/test_image_transport_decompressor.cpp
  )
  target_link_libraries(test_image_transport_decompressor
    ${PROJECT_NAME}
  )

  ament_add_ros_isolated_gtest(test_image_transport_decompressor_node
    test/test_image_transport_decompressor_node.cpp
  )
  target_link_libraries(test_image_transport_decompressor_node
    ${PROJECT_NAME}_ros
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
