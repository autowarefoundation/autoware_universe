# Custom install script to copy acados headers at install time
# This ensures the headers are available after acados is built

# Get the acados include directory from the parent scope
set(ACADOS_INCLUDE_DIR "${ACADOS_INCLUDE_DIR}")
set(INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/install/autoware_path_optimizer/include")

message(STATUS "Installing acados headers from: ${ACADOS_INCLUDE_DIR}")
message(STATUS "Installing acados headers to: ${INSTALL_INCLUDE_DIR}")

if(EXISTS "${ACADOS_INCLUDE_DIR}")
  file(INSTALL "${ACADOS_INCLUDE_DIR}/"
    DESTINATION "${INSTALL_INCLUDE_DIR}"
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
  )
  message(STATUS "Acados headers installed successfully")
else()
  message(WARNING "Acados include directory does not exist: ${ACADOS_INCLUDE_DIR}")
endif()
