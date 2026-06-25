cmake_minimum_required(VERSION 3.14)
project(autoware_generic_value_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/generic_value_converter_node.cpp
  src/value_map.cpp
  src/csv_loader.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::generic_value_converter::GenericValueConverterNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_csv_loader
    test/test_csv_loader.cpp
  )
  if(TARGET test_csv_loader)
    target_link_libraries(test_csv_loader ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_csv_loader
      rclcpp
    )
  endif()

  ament_add_gtest(test_value_map
    test/test_value_map.cpp
  )
  if(TARGET test_value_map)
    target_link_libraries(test_value_map ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_value_map
      rclcpp
    )
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    data
    launch
    schema
)
