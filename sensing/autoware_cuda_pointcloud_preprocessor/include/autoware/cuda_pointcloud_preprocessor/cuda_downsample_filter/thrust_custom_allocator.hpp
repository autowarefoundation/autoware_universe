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

  explicit ThrustCustomAllocator(cudaStream_t stream) : stream_(stream) {}

  pointer allocate(size_type num)
  {
    uint8_t * buffer(nullptr);
    auto result = cudaMallocAsync(&buffer, num, stream_);
    if (result != ::cudaSuccess) {
      std::stringstream s;
      s << ::cudaGetErrorName(result) << " : " << ::cudaGetErrorString(result);
      throw std::runtime_error{s.str()};
    }

    cudaMemsetAsync(buffer, 0, num, stream_);

    return pointer(thrust::device_pointer_cast(buffer));
  }

  void deallocate(pointer ptr, size_t)
  {
    auto result = cudaFreeAsync(thrust::raw_pointer_cast(ptr), stream_);
    if (result != ::cudaSuccess) {
      std::stringstream s;
      s << ::cudaGetErrorName(result) << " : " << ::cudaGetErrorString(result);
      throw std::runtime_error{s.str()};
    }
  }

private:
  cudaStream_t stream_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__THRUST_CUSTOM_ALLOCATOR_HPP_
