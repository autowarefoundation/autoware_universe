cmake_minimum_required(VERSION 3.5)
project(autoware_speed_scale_corrector)

find_package(autoware_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
autoware_package()

ament_auto_add_library(autoware_speed_scale_corrector_lib SHARED
  src/node.cpp
  src/utils.cpp
  src/speed_scale_estimator.cpp
)

# as a ros2 node
ament_auto_add_executable(autoware_speed_scale_corrector src/main.cpp)
target_link_libraries(autoware_speed_scale_corrector
  autoware_speed_scale_corrector_lib
  ${glog_LIBRARIES}
)
target_include_directories(autoware_speed_scale_corrector PRIVATE ${glog_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_autoware_speed_scale_corrector
    test/test_speed_scale_estimator.cpp
    test/test_utils.cpp
  )
  target_link_libraries(test_autoware_speed_scale_corrector autoware_speed_scale_corrector_lib)
  ament_target_dependencies(test_autoware_speed_scale_corrector ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endif()

# Install launch files
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
