// Copyright 2025 Tier IV, Inc.
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

#include "autoware/cuda_utils/cuda_check_error.hpp"
#include "autoware/cuda_utils/cuda_gtest_utils.hpp"
#include "autoware/cuda_utils/cuda_utils.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

TEST(CudaUtilsTest, ClearAsyncFunction)
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();

  // Create a CUDA stream for testing
  cudaStream_t stream{};
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream));

  // Allocate some device memory
  float * device_ptr{};
  CHECK_CUDA_ERROR(
    cudaMalloc(reinterpret_cast<void **>(&device_ptr), 100 * sizeof(float)));  // NOLINT

  // Test the clear_async function
  EXPECT_NO_THROW(autoware::cuda_utils::clear_async(device_ptr, 100, stream));

  // Synchronize to ensure the operation completed
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));

  // Clean up
  CHECK_CUDA_ERROR(cudaFree(device_ptr));
  CHECK_CUDA_ERROR(cudaStreamDestroy(stream));
}
