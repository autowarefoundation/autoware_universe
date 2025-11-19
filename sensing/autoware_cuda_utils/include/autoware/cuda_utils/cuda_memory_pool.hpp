// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This code is licensed under CC0 1.0 Universal (Public Domain).
// You can use this without any limitation.
// https://creativecommons.org/publicdomain/zero/1.0/deed.en
// borrowed from https://proc-cpuinfo.fixstars.com/2019/02/cuda_smart_pointer/

#ifndef AUTOWARE__CUDA_UTILS__CUDA_MEMORY_POOL_HPP_
#define AUTOWARE__CUDA_UTILS__CUDA_MEMORY_POOL_HPP_

#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <cuda_runtime_api.h>

namespace autoware::cuda_utils
{

/** \brief Create a CUDA memory pool.
 *
 * This function creates a CUDA memory pool that can be used to allocate
 * small blocks of memory quickly and efficiently.
 *
 * \param max_mem_pool_size_in_byte The maximum size of the memory pool in bytes. WHen more than the
 * this value of memory are held by the memory pool, the allocator will try to release memory.
 * \param device_id The ID of the CUDA device to create the memory pool on.
 * \return A CUDA memory pool handler.
 */
cudaMemPool_t create_memory_pool(const size_t & max_mem_pool_size_in_byte, const int device_id)
{
  cudaMemPool_t mem_pool;

  cudaMemPoolProps pool_props = {};
  pool_props.allocType = cudaMemAllocationTypePinned;
  pool_props.location.id = device_id;
  pool_props.location.type = cudaMemLocationTypeDevice;
  CHECK_CUDA_ERROR(cudaMemPoolCreate(&mem_pool, &pool_props));

  // Configure the memory pool reusing allocation
  // we set a high release threshold so that the allocated memory region will be reused
  uint64_t pool_release_threshold = max_mem_pool_size_in_byte;
  CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
    mem_pool, cudaMemPoolAttrReleaseThreshold, static_cast<void *>(&pool_release_threshold)));

  return mem_pool;
}

}  // namespace autoware::cuda_utils

#endif  // AUTOWARE__CUDA_UTILS__CUDA_MEMORY_POOL_HPP_
