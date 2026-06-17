if (CMAKE_VERSION VERSION_LESS 3.12)
  cmake_minimum_required(VERSION 3.8)
else()
  cmake_minimum_required(VERSION 3.8...3.24)
endif()

project(cnpy LANGUAGES C CXX)

if (NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable -Wno-unused-but-set-variable")
set(LIBRARY_NAME ${PROJECT_NAME})
option(ENABLE_STATIC "Build static (.a) library" ON)

find_package(ZLIB REQUIRED)

include_directories(${ZLIB_INCLUDE_DIRS})

add_library(${LIBRARY_NAME} SHARED "cnpy.cpp")
target_link_libraries(cnpy ${ZLIB_LIBRARIES})

set(LIB_INSTALL_DIR "lib/${PROJECT_NAME}")
if (WIN32)
  install(TARGETS ${LIBRARY_NAME}
      EXPORT ${PROJECT_NAME}-targets
      RUNTIME DESTINATION ${LIB_INSTALL_DIR}
      PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
else()
  install(TARGETS ${LIBRARY_NAME}
      EXPORT ${PROJECT_NAME}-targets
      LIBRARY DESTINATION ${LIB_INSTALL_DIR}
      PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
endif()
if(ENABLE_STATIC)
    add_library(${LIBRARY_NAME}-static STATIC "cnpy.cpp")
    set_target_properties(cnpy-static PROPERTIES OUTPUT_NAME "cnpy")
    install(TARGETS "cnpy-static" ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
endif(ENABLE_STATIC)

install(FILES "cnpy.h"
    DESTINATION include/${PROJECT_NAME})
install(FILES "mat2npz" "npy2mat" "npz2mat"
    DESTINATION bin
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

add_executable(example1 example1.cpp)
target_link_libraries(example1 cnpy)

# Export project export set to the build tree
export(
  EXPORT ${PROJECT_NAME}-targets
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
  )

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry ~/.cmake)
export(PACKAGE ${PROJECT_NAME})

include(CMakePackageConfigHelpers)

# BUILD CONFIG
set(TMP_INCLUDE_INSTALL_DIR ${PROJECT_SOURCE_DIR})
configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/Modules/project-config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${PROJECT_BINARY_DIR}
  PATH_VARS TMP_INCLUDE_INSTALL_DIR
  )

# INSTALL CONFIG
set(CMAKE_CONFIG_DEST "lib/cmake/${PROJECT_NAME}")
set(TMP_INCLUDE_INSTALL_DIR include/${PROJECT_NAME})
configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/Modules/project-config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${CMAKE_CONFIG_DEST}
  PATH_VARS TMP_INCLUDE_INSTALL_DIR
  )

install(
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION ${CMAKE_CONFIG_DEST}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  # ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}-config-version.cmake
  DESTINATION ${CMAKE_CONFIG_DEST}
)
