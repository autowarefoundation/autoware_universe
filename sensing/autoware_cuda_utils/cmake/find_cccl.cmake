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

# CUDA 13 moved the CCCL headers (Thrust, CUB, libcu++) to include/cccl/.
# Only nvcc adds that directory to its include path. The CCCL CMake config
# is outside the default search path, so find_package needs the hints.
# find_package searches the hints after CCCL_ROOT and CMAKE_PREFIX_PATH but
# before the system prefixes, so the toolkit config wins over a system copy
# and explicit user overrides still win over the hints.

find_package(CUDAToolkit REQUIRED)
find_package(CCCL CONFIG REQUIRED HINTS
  "${CUDAToolkit_LIBRARY_DIR}/cmake"
  "${CUDAToolkit_TARGET_DIR}/lib64/cmake"
  "${CUDAToolkit_TARGET_DIR}/lib/cmake")
