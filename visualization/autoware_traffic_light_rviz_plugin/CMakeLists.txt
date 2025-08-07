cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

# Enable Qt MOC and include current dir for Qt headers
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

# Recursively collect all .cpp and .hpp files
file(GLOB_RECURSE SRC_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
file(GLOB_RECURSE HDR_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.hpp"
)

# Create plugin library from all sources
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${HDR_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

# Suppress specific warnings if desired
# Treat OGRE headers as system headers to suppress warnings
if(TARGET rviz_ogre_vendor::OgreMain)
  get_target_property(OGRE_INCLUDE_DIRS rviz_ogre_vendor::OgreMain INTERFACE_INCLUDE_DIRECTORIES)
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${OGRE_INCLUDE_DIRS})
endif()

# Export plugin description for RViz
pluginlib_export_plugin_description_file(
  rviz_common
  plugins/plugin_description.xml
)

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
