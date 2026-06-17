if (CMAKE_VERSION VERSION_LESS 3.12)
  cmake_minimum_required(VERSION 3.8)
else()
  cmake_minimum_required(VERSION 3.8...3.24)
endif()

# MPPI sets -gencode via CMAKE_CUDA_FLAGS; disable CMake's default sm_52 codegen.
set(CMAKE_CUDA_ARCHITECTURES OFF)

project(mppi-path-tracking LANGUAGES C CXX CUDA)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(MPT_BUILD_EXAMPLES "Build path-tracking CUDA examples." OFF)
option(MPT_BUILD_TESTS "Build path-tracking GPU unit tests." OFF)

# MPPI-Generic is built as a dependency only (no upstream examples/tests).
set(MPPI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MPPI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# Configure CUDA arch flags at the top level so sibling targets (examples/) inherit them.
# When MPPI-Generic is add_subdirectory'd, its cmake only applies flags inside lib/.
include(MPPIGenericToolsConfig)

add_subdirectory(lib/MPPI-Generic)

# Project extensions live in include/ and are searched before lib/MPPI-Generic/include.
add_library(mppi_path_tracking INTERFACE)
add_library(MppiPathTracking::extensions ALIAS mppi_path_tracking)
target_include_directories(mppi_path_tracking BEFORE INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_link_libraries(mppi_path_tracking INTERFACE MPPI::MPPI)

if (MPT_BUILD_EXAMPLES OR MPT_BUILD_TESTS)
  add_subdirectory(src)
endif()

if (MPT_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if (MPT_BUILD_TESTS)
  add_subdirectory(tests)
endif()
