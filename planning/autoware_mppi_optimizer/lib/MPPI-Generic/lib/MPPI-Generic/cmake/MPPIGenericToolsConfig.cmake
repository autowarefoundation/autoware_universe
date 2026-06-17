if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type options: Release, RelWithDebInfo, Debug" FORCE)
  message(STATUS "Setting Build Type to ${CMAKE_BUILD_TYPE} by default")
endif()

if(NOT DEFINED CMAKE_CXX_STANDARD)
  if (CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL "13")
    # Cuda's libcu++ requires C++17 on CUDA 13+
    set(CMAKE_CXX_STANDARD 17)
  else()
    set(CMAKE_CXX_STANDARD 11)
  endif()
endif()
if (NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD ${CMAKE_CXX_STANDARD})
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Look for ccache to potentially speed up repeated compilations
find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM AND NOT MPPI_USE_CCACHE)
  message(STATUS "Using ccache to speed up repeated builds")
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_CUDA_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
elseif(NOT CCACHE_PROGRAM AND NOT MPPI_USE_CCACHE)
  message(STATUS "ccache not found. Using ccache can speed up repeated builds.")
endif()

# Add debug flags so cuda-gdb can be used to stop inside a kernel.
# NOTE: You may have to run make multiple times for it to compile successfully.
set(CMAKE_CUDA_FLAGS_DEBUG "${CMAKE_CUDA_FLAGS_DEBUG} -G --keep")
set(CMAKE_CUDA_FLAGS_RELWITHDEBINFO "${CMAKE_CUDA_FLAGS_RELWITHDEBINFO} --generate-line-info")

# Generate variable for all the extra cuda libraries we use
set(MPPI_GENERIC_CUDA_EXTRA_LIBS "")
if (CMAKE_VERSION VERSION_LESS 3.24)
  # required for curand on some systems
  find_package(CUDA REQUIRED)

  if(${CUDA_curand_LIBRARY} MATCHES "NOTFOUND")
      message(ERROR "cuRAND library not found.")
  endif()

  list(APPEND MPPI_GENERIC_CUDA_EXTRA_LIBS ${CUDA_curand_LIBRARY} ${CUDA_CUFFT_LIBRARIES})
else()
  find_package(CUDAToolkit REQUIRED)
  set(CUDA_VERSION ${CUDAToolkit_VERSION})
  list(APPEND MPPI_GENERIC_CUDA_EXTRA_LIBS CUDA::curand CUDA::cufft)
endif()

# Generate name for MPPI header library
set(MPPI_HEADER_LIBRARY_NAME mppi_header_only_lib)

set(CUDA_PROPAGATE_HOST_FLAGS OFF)

#################################################################
# Autodetect Cuda Architecture on system and add to executables #
#################################################################
# CMake 3.24 added '-arch=native' support so until that version, we need to use the old method of autodetection
if (CMAKE_VERSION VERSION_LESS 3.24)
  # Don't rerun autodetection when used as a submodule
  if (NOT DEFINED MPPI_ARCH_FLAGS)
    # More info for autodetection:
    # https://stackoverflow.com/questions/35485087/determining-which-gencode-compute-arch-values-i-need-for-nvcc-within-cmak
    CUDA_SELECT_NVCC_ARCH_FLAGS(MPPI_ARCH_FLAGS ${MPPI_CUDA_ARCH_LIST})

    if (MPPI_ARCH_FLAGS STREQUAL "")
      set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -maxrregcount=32 -arch=sm_35")
    else()
      string(REGEX REPLACE "-gencode;arch" "-gencode=arch" MPPI_ARCH_FLAGS "${MPPI_ARCH_FLAGS}")
      string(REPLACE ";" " " MPPI_ARCH_FLAGS "${MPPI_ARCH_FLAGS}")
      # string(REGEX REPLACE "^-gencode=arch" "-arch" MPPI_ARCH_FLAGS "${MPPI_ARCH_FLAGS}")
      message(STATUS "CUDA Architecture(s): ${MPPI_ARCH_FLAGS}")
      set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} ${MPPI_ARCH_FLAGS}")
    endif()
  else()
    message(STATUS "Autodetection already ran and found ${MPPI_ARCH_FLAGS}.")
  endif()
else()
  # Don't rerun autodetection when used as a submodule
  if (NOT DEFINED MPPI_ARCH_FLAGS)
    if ("${MPPI_CUDA_ARCH_LIST}" STREQUAL "")
      set(CMAKE_CUDA_ARCHITECTURES native)
    else()
      set(CMAKE_CUDA_ARCHITECTURES "${MPPI_CUDA_ARCH_LIST}")
    endif()
    set(MPPI_ARCH_FLAGS "${CMAKE_CUDA_ARCHITECTURES}")
    message(STATUS "CUDA Architecture(s): ${CMAKE_CUDA_ARCHITECTURES}")
  else()
    message(STATUS "Autodetection already ran and found ${MPPI_ARCH_FLAGS}.")
  endif()
endif()
