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

#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_MEMPOOL_WRAPPER_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_MEMPOOL_WRAPPER_HPP_

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <cuda_runtime_api.h>

#include <utility>

namespace autoware
{

class CudaMempool
{
public:
  inline explicit CudaMempool(int device_id = 0, int64_t pool_release_threshold = 0)
  {
    cudaMemPoolProps pool_props = {};

    // Check if the device is valid
    CHECK_CUDA_ERROR(cudaGetDevice(&device_id));

    pool_props.allocType = cudaMemAllocationTypePinned;
    pool_props.location.id = device_id;
    pool_props.location.type = cudaMemLocationTypeDevice;

    CHECK_CUDA_ERROR(cudaMemPoolCreate(&pool_, &pool_props));

    if (pool_release_threshold >= 0) {
      CHECK_CUDA_ERROR(
        cudaMemPoolSetAttribute(pool_, cudaMemPoolAttrReleaseThreshold, &pool_release_threshold));
    }
  }

  CudaMempool(const CudaMempool &) = delete;
  inline CudaMempool(CudaMempool && other) : pool_(std::move(other.pool_))
  {
    other.pool_ = nullptr;
  }

  CudaMempool & operator=(const CudaMempool &) = delete;
  inline CudaMempool & operator=(CudaMempool && other)
  {
    if (this != &other) {
      release();
      pool_ = std::move(other.pool_);
      other.pool_ = nullptr;
    }

    return *this;
  }

  inline cudaMemPool_t get() { return pool_; }

  ~CudaMempool() { release(); }

private:
  inline void release()
  {
    if (pool_) {
      CHECK_CUDA_ERROR(cudaMemPoolDestroy(pool_));
    }

    pool_ = nullptr;
  }

  cudaMemPool_t pool_;
};
}  // namespace autoware

#endif  // AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_MEMPOOL_WRAPPER_HPP_
