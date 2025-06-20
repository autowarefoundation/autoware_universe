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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__THRUST_CUSTOM_ALLOCATOR_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__THRUST_CUSTOM_ALLOCATOR_HPP_

#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <cuda_runtime.h>
#include <thrust/device_malloc_allocator.h>

#include <sstream>

namespace autoware::cuda_pointcloud_preprocessor
{
struct ThrustCustomAllocator : public thrust::device_malloc_allocator<uint8_t>
{
  // ref: https://stackoverflow.com/questions/76594790/memory-pool-in-thrust-execution-policy
public:
  using Base = thrust::device_malloc_allocator<uint8_t>;
  using pointer = typename Base::pointer;
  using size_type = typename Base::size_type;

  explicit ThrustCustomAllocator(cudaStream_t stream, cudaMemPool_t & mem_pool)
  : stream_(stream), mem_pool_(mem_pool)
  {
  }

  pointer allocate(size_type num)
  {
    uint8_t * buffer(nullptr);
    CHECK_CUDA_ERROR(cudaMallocFromPoolAsync(&buffer, num, mem_pool_, stream_));

    cudaMemsetAsync(buffer, 0, num, stream_);

    return pointer(thrust::device_pointer_cast(buffer));
  }

  void deallocate(pointer ptr, size_t)
  {
    CHECK_CUDA_ERROR(cudaFreeAsync(thrust::raw_pointer_cast(ptr), stream_));
  }

private:
  cudaStream_t stream_;
  cudaMemPool_t mem_pool_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__THRUST_CUSTOM_ALLOCATOR_HPP_
