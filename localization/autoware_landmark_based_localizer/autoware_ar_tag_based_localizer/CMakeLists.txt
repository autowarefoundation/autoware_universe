cmake_minimum_required(VERSION 3.14)
project(autoware_ar_tag_based_localizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ar_tag_based_localizer.cpp
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)
if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  find_package(agnocastlib REQUIRED)
  ament_target_dependencies(${PROJECT_NAME} agnocastlib)
endif()
autoware_agnocast_wrapper_setup(${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "ArTagBasedLocalizer"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test.cpp
    src/ar_tag_based_localizer.cpp
  )
  target_include_directories(test_${PROJECT_NAME}
    SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_${PROJECT_NAME}  ${OpenCV_LIBRARIES})
  ament_target_dependencies(test_${PROJECT_NAME} autoware_agnocast_wrapper)
  if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    ament_target_dependencies(test_${PROJECT_NAME} agnocastlib)
  endif()
  autoware_agnocast_wrapper_setup(test_${PROJECT_NAME})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
