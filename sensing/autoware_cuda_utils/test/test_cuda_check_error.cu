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

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <stdexcept>

TEST(CudaCheckErrorTest, MacroUsageWithSuccess)
{
  // Test the CHECK_CUDA_ERROR macro with success - should not throw
  EXPECT_NO_THROW(CHECK_CUDA_ERROR(cudaSuccess));
}

TEST(CudaCheckErrorTest, MacroUsageWithError)
{
  // Test the CHECK_CUDA_ERROR macro with error - should throw
  EXPECT_THROW(CHECK_CUDA_ERROR(cudaErrorInvalidValue), std::runtime_error);
}
