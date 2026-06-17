if (CMAKE_VERSION VERSION_LESS 3.12)
  cmake_minimum_required(VERSION 3.8)
else()
  cmake_minimum_required(VERSION 3.8...3.24)
endif()

# https://cmake.org/cmake/help/latest/policy/CMP0104.html
if (CMAKE_VERSION VERSION_LESS 3.24 AND POLICY CMP0104)
    cmake_policy(SET CMP0104 OLD)
endif()
# Get Version from file
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" MPPI_VERSION_FROM_FILE)
# Clean up trailng whitespace and newlines
string(STRIP "${MPPI_VERSION_FROM_FILE}" MPPI_VERSION)
project(MPPI-Generic VERSION ${MPPI_VERSION} LANGUAGES C CXX CUDA)

####################################
# Set up options for configuration #
####################################
option(MPPI_BUILD_TESTS "Build unit tests." OFF)
option(MPPI_BUILD_EXAMPLES "Build example code." OFF)
option(MPPI_USE_CCACHE "Use ccache (when available) to potentially speed up repeated builds" ON)
option(MPPI_USE_CUDA_BARRIERS "Compile MPPI-Generic with cuda barrier support. Turn off for GPUs older than the 20 series." ON)
option(MPPI_USE_CUDA_BARRIERS_COST "Compile MPPI-Generic with cuda barriers for cost-only kernels when general barrier support is enabled." OFF)
option(MPPI_USE_CUDA_BARRIERS_DYN "Compile MPPI-Generic with cuda barriers for dynamics-only kernels when general barrier support is enabled." ON)
option(MPPI_USE_CUDA_BARRIERS_ROLLOUT "Compile MPPI-Generic with cuda barriers for combined kernels when general barrier support is enabled." ON)
option(MPPI_EXPORT_PACKAGE "Export this folder as the MPPI installation to the global CMake package repository" OFF)
set(MPPI_CUDA_ARCH_LIST "" CACHE STRING "Specific CUDA Architectures to build for. Leave empty for automatic selection.")

# Configure CMake Cuda Flags for MPPI
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(MPPIGenericToolsConfig)

set(BUILD_FLAGS "-Wuninitialized -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -w")

# Allow the lib location to be overwritten from command line
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    get_filename_component(PROJECT_LIBS_DIR ${PROJECT_BINARY_DIR}/lib ABSOLUTE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
else()
    get_filename_component(PROJECT_LIBS_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ABSOLUTE)
endif()

# set up cnpy
# Don't install cnpy to user package registry
set(TMP_PACKAGE_REGISTRY_EXPORT ${CMAKE_EXPORT_NO_PACKAGE_REGISTRY})
set(CMAKE_EXPORT_NO_PACKAGE_REGISTRY ON)
add_subdirectory(submodules/cnpy)
# Set the no package registry back to default
set(CMAKE_EXPORT_NO_PACKAGE_REGISTRY ${TMP_PACKAGE_REGISTRY_EXPORT})

# Find Eigen
find_package(Eigen3 REQUIRED)

# REQUIRED for CUDA to correctly align Eigen member variables inside structures/classes
# https://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2016/06/msg00006.html
# add_definitions(-DEIGEN_MAX_STATIC_ALIGN_BYTES=0)

# Generate Version Header File from Project Version
configure_file("${PROJECT_SOURCE_DIR}/include/mppi/version.h.in" "${PROJECT_BINARY_DIR}/include/mppi/version.h")

# Create a Header-only MPPI Library
add_library(${MPPI_HEADER_LIBRARY_NAME} INTERFACE)
add_library(MPPI::MPPI ALIAS ${MPPI_HEADER_LIBRARY_NAME})
target_include_directories(${MPPI_HEADER_LIBRARY_NAME} INTERFACE
  ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(${MPPI_HEADER_LIBRARY_NAME} INTERFACE
  ${MPPI_GENERIC_CUDA_EXTRA_LIBS}
  ${CUDA_LIBRARIES}
  cnpy
  Eigen3::Eigen
)
target_include_directories(${MPPI_HEADER_LIBRARY_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/submodules/cnpy>"
  "$<INSTALL_INTERFACE:include/cnpy>"
)

# --display-error-number was added as a compilation flag in CUDA 11.2
if (${CUDA_VERSION} VERSION_GREATER_EQUAL "11.2")
target_compile_options(${MPPI_HEADER_LIBRARY_NAME} INTERFACE
  $<$<COMPILE_LANGUAGE:CUDA>:--display-error-number>
)
endif()

# Allow the ability to turn off cuda barriers through CMake. Necessary for GPUs older than
# the 20 series
if (MPPI_USE_CUDA_BARRIERS)
  target_compile_definitions(${MPPI_HEADER_LIBRARY_NAME} INTERFACE
    CMAKE_USE_CUDA_BARRIERS)
  if (MPPI_USE_CUDA_BARRIERS_DYN)
    target_compile_definitions(${MPPI_HEADER_LIBRARY_NAME} INTERFACE
      CMAKE_USE_CUDA_BARRIERS_DYN)
  endif()
  if (MPPI_USE_CUDA_BARRIERS_ROLLOUT)
    target_compile_definitions(${MPPI_HEADER_LIBRARY_NAME} INTERFACE
      CMAKE_USE_CUDA_BARRIERS_ROLLOUT)
  endif()
  if (MPPI_USE_CUDA_BARRIERS_COST)
    target_compile_definitions(${MPPI_HEADER_LIBRARY_NAME} INTERFACE
      CMAKE_USE_CUDA_BARRIERS_COST)
  endif()
endif()

if (MPPI_BUILD_TESTS OR MPPI_BUILD_EXAMPLES)
    # find yaml-cpp
    find_package(yaml-cpp REQUIRED)
    include_directories(${YAML_CPP_INCLUDE_DIR})
endif()

# Install all library header files
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/mppi
  DESTINATION include
  PATTERN "*.in" EXCLUDE
)

# Install generated header file version.h
install(
  DIRECTORY ${PROJECT_BINARY_DIR}/include/mppi
  DESTINATION include
)

###########################################
# Set up CMake configuration installation #
###########################################
set(CMAKE_CONFIG_DEST "lib/cmake/${PROJECT_NAME}")
install(
  TARGETS ${MPPI_HEADER_LIBRARY_NAME}
  EXPORT ${PROJECT_NAME}-targets
)

install(
  EXPORT ${PROJECT_NAME}-targets
  NAMESPACE MPPI::
  DESTINATION ${CMAKE_CONFIG_DEST}
  FILE ${PROJECT_NAME}Targets.cmake
)

install(
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION ${CMAKE_CONFIG_DEST}
  FILE ${PROJECT_NAME}TargetsNoNamespace.cmake
)

# Set up MPPI to export the header library as MPPI in the MPPI namespace
set_target_properties (${MPPI_HEADER_LIBRARY_NAME} PROPERTIES EXPORT_NAME MPPI)
export (TARGETS ${MPPI_HEADER_LIBRARY_NAME}
        NAMESPACE MPPI::
        FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry ~/.cmake)
if (MPPI_EXPORT_PACKAGE)
  export(PACKAGE ${PROJECT_NAME})
endif()

#################################################################################
# Create CMake Config and Version files and install in the appropriate location #
#################################################################################

# Create a CMake variable of the appropriate include directory depending on
# whether it is consider from the build tree or the install tree
set(TMP_MPPI_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
set(TMP_MPPI_TARGET_CMAKE_FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")
# Set up build tree config file
configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${PROJECT_BINARY_DIR}
  PATH_VARS TMP_MPPI_INCLUDE_DIRS TMP_MPPI_TARGET_CMAKE_FILE
)

# Set up install tree config file
set(TMP_MPPI_INCLUDE_DIRS "include/")
set(TMP_MPPI_TARGET_CMAKE_FILE "${CMAKE_CONFIG_DEST}/${PROJECT_NAME}Targets.cmake")
configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in
  ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_CONFIG_DEST}
  PATH_VARS TMP_MPPI_INCLUDE_DIRS TMP_MPPI_TARGET_CMAKE_FILE
)

write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMinorVersion
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MPPIGenericToolsConfig.cmake
  ${PROJECT_BINARY_DIR}/MPPIGenericToolsConfig.cmake
  COPYONLY
)

install(FILES
  ${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/MPPIGenericToolsConfig.cmake
  DESTINATION ${CMAKE_CONFIG_DEST}
)

if (MPPI_BUILD_TESTS OR MPPI_BUILD_EXAMPLES)
    add_subdirectory(src)
endif()
if (MPPI_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Add custom cmake finds
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

# Use CMake to download gtest as part of the configure step
###################################################################
# Add gtest
###################################################################
if (NOT DEFINED CMAKE_TOOLCHAIN_FILE AND MPPI_BUILD_TESTS)
  message(STATUS "Building MPPI Generic tests")
  enable_testing()
  ############################################################
  # copied from
  # https://github.com/google/googletest/tree/master/googletest#incorporating-into-an-existing-cmake-project
  ############################################################
  # Download and unpack googletest at configure time
  if (NOT TARGET gtest_main)
    list(GET CMAKE_MODULE_PATH -1 MPPI_GENERIC_MODULES)
    configure_file(${MPPI_GENERIC_MODULES}/CMakeLists.txt.gtest.in
                   ${PROJECT_BINARY_DIR}/googletest-download/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
      RESULT_VARIABLE result
      WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/googletest-download )
    if(result)
      message(FATAL_ERROR "CMake step for googletest failed: ${result}")
    endif()
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
      RESULT_VARIABLE result
      WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/googletest-download )
    if(result)
      message(FATAL_ERROR "Build step for googletest failed: ${result}")
    endif()

    # Prevent overriding the parent project's compiler/linker
    # settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Prevent googletest installation
    set(INSTALL_GTEST OFF)

    # Add googletest directly to our build. This defines
    # the gtest and gtest_main targets.
    add_subdirectory(${PROJECT_BINARY_DIR}/googletest-src
                     ${PROJECT_BINARY_DIR}/googletest-build)
  endif()

  include(GoogleTest)
  add_subdirectory(tests)
endif()

# Uninstall
if(NOT TARGET uninstall)
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

  add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
