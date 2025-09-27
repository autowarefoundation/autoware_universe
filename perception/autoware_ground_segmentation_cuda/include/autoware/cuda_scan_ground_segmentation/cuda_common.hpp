// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_COMMON_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_COMMON_HPP_

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <cuda_runtime.h>

#ifndef CUDAH
#define CUDAH __forceinline__ __host__ __device__
#endif

#ifndef BLOCK_SIZE_X
#define BLOCK_SIZE_X (256)
#endif

#ifndef WARP_SIZE
#define WARP_SIZE (32)
#endif

#ifndef FULL_MASK
#define FULL_MASK (0xFFFFFFFF)
#endif

#endif  // AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_COMMON_HPP_
