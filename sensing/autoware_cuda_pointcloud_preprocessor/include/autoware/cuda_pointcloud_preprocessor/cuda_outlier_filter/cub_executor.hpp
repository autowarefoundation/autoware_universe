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

/* *INDENT-OFF* */
#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUB_EXECUTOR_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUB_EXECUTOR_HPP_

#include "autoware/cuda_utils/cuda_check_error.hpp"
#include "autoware/cuda_utils/cuda_unique_ptr.hpp"

#include <cuda_runtime.h>

#include <utility>

namespace autoware::cuda_pointcloud_preprocessor
{
class CubExecutor
{
  /**
   * Thin wrapper class for CUB operations to make them easy for repeated use
   * while keeping fine-grained control
   */
  template <typename T>
  using CudaPooledUniquePtr = autoware::cuda_utils::CudaPooledUniquePtr<T>;

public:
  /**
   * \brief Updates the temporary storage buffer, allocating if necessary.
   *
   * This function checks if the current temporary storage is large enough to
   * accommodate the requested size. If not, it allocates a new buffer with the
   * requested size and updates the internal state.
   *
   * \param new_size   The requested size of the temporary storage in bytes.
   * \param stream     The CUDA stream to associate with the allocation.
   * \param mem_pool   The CUDA memory pool to use for allocation.
   */
  void update_temp_storage(const size_t & new_size, cudaStream_t & stream, cudaMemPool_t & mem_pool)
  {
    if (current_temp_storage_bytes_ < new_size) {
      temp_storage_ = autoware::cuda_utils::make_unique<uint8_t>(new_size, stream, mem_pool);
      current_temp_storage_bytes_ = new_size;
    }
  }

  /**
   * \brief Executes a `CubLambda` with temporary storage allocated from a CUDA memory pool.
   *
   * First queries the CUB API function through lambda for its temporary storage requirements, then
   * allocates the storage (if needed) and finally executes the CUB API (lambda) with the allocated
   * storage.
   *
   * \param cub_lambda The lambda wrapping CUB API so that type deduction occurs inside this
   *                   function.
   * \param stream CUDA stream for execution.
   * \param mem_pool CUDA memory pool for temporary storage allocation.
   * \param args Arguments to be passed to the callable.
   *
   * NOTE:
   * cub_lambda looks like:
   * ```c++
   * auto cub_lambda = [](auto &&... args) {
   *   return <CUB_API_TO_CALL>(std::forward<decltype(args)>(args)...);
   * };
   * ```
   */
  template <typename CubLambda, typename... Args>
  void run_with_temp_storage(
    CubLambda cub_lambda, cudaStream_t & stream, cudaMemPool_t & mem_pool, Args &&... args)
  {
    // Query temp storage size
    size_t temp_storage_bytes = 0;
    CHECK_CUDA_ERROR(cub_lambda(nullptr, temp_storage_bytes, std::forward<Args>(args)...));

    // Allocate temporary storage if needed
    update_temp_storage(temp_storage_bytes, stream, mem_pool);

    // Execute operation
    CHECK_CUDA_ERROR(
      cub_lambda(temp_storage_.get(), temp_storage_bytes, std::forward<Args>(args)...));
  }

private:
  size_t current_temp_storage_bytes_{0};
  CudaPooledUniquePtr<uint8_t> temp_storage_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUB_EXECUTOR_HPP_
/* *INDENT-ON* */
