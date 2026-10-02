# Copyright 2026 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# CUDA 13 moved the CCCL headers to include/cccl/. The CCCL CMake config sits
# outside the default CMake search path, so find_package needs the hints.

find_package(CUDAToolkit REQUIRED)
find_package(CCCL CONFIG REQUIRED HINTS
  "${CUDAToolkit_LIBRARY_DIR}/cmake"
  "${CUDAToolkit_TARGET_DIR}/lib64/cmake"
  "${CUDAToolkit_TARGET_DIR}/lib/cmake")

# The CCCL targets are not imported, so this marker re-marks their include
# directory as a system include for consumers.
if(NOT TARGET autoware_cuda_utils::cccl_system)
  add_library(autoware_cuda_utils::cccl_system INTERFACE IMPORTED GLOBAL)
  get_target_property(_autoware_cuda_utils_cccl_dir CCCL::libcudacxx
    INTERFACE_INCLUDE_DIRECTORIES)
  if(_autoware_cuda_utils_cccl_dir)
    set_property(TARGET autoware_cuda_utils::cccl_system PROPERTY
      INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_autoware_cuda_utils_cccl_dir}")
  endif()
  unset(_autoware_cuda_utils_cccl_dir)
endif()
