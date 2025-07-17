// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE__CUDA_UTILS__CUDA_GTEST_UTILS_HPP_
#define AUTOWARE__CUDA_UTILS__CUDA_GTEST_UTILS_HPP_

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#define SKIP_TEST_IF_CUDA_UNAVAILABLE()                              \
  if (!autoware::cuda_utils::is_cuda_runtime_available()) {          \
    GTEST_SKIP() << "CUDA runtime is not available. Skipping test."; \
  }

namespace autoware::cuda_utils
{

/**
 * @brief Check if CUDA is available at runtime.
 * @return true if CUDA is available, false otherwise.
 */
inline bool is_cuda_runtime_available()
{
  int device_count = 0;
  cudaError_t error = cudaGetDeviceCount(&device_count);
  return (error == cudaSuccess && device_count > 0);
}

/**
 * @brief Base class for CUDA-related tests that runs only if CUDA is available.
 *
 * Usage: Inherit from this class and use the child class as a test fixture.
 * Example:
 *
 * ```c++
 * class MyTestSuite : public autoware::cuda_utils::CudaTest {};
 *
 * TEST_F(MyTestSuite, MyTest) {
 *   // Your test code here
 * }
 * ```
 */
class CudaTest : public ::testing::Test
{
protected:
  void SetUp() override { SKIP_TEST_IF_CUDA_UNAVAILABLE(); }
};

}  // namespace autoware::cuda_utils

#endif  // AUTOWARE__CUDA_UTILS__CUDA_GTEST_UTILS_HPP_
