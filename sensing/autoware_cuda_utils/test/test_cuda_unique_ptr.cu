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
#include "autoware/cuda_utils/cuda_unique_ptr.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

class CudaUniquePtrTest : public autoware::cuda_utils::CudaTest
{
};

TEST_F(CudaUniquePtrTest, MakeUniqueDeviceMemory)
{
  // Test creating a single object on device
  auto ptr = autoware::cuda_utils::make_unique<float>();
  EXPECT_NE(ptr.get(), nullptr);
}

TEST_F(CudaUniquePtrTest, MakeUniqueDeviceArray)
{
  // Test creating an array on device
  auto ptr = autoware::cuda_utils::make_unique<float[]>(100);
  EXPECT_NE(ptr.get(), nullptr);

  cudaPointerAttributes attributes{};
  CHECK_CUDA_ERROR(cudaPointerGetAttributes(&attributes, ptr.get()));
  EXPECT_EQ(attributes.type, cudaMemoryTypeDevice);
  EXPECT_EQ(attributes.devicePointer, ptr.get());
}

TEST_F(CudaUniquePtrTest, MakeUniqueHostMemory)
{
  // Test creating a single object on host
  auto ptr = autoware::cuda_utils::make_unique_host<float>();
  EXPECT_NE(ptr.get(), nullptr);

  cudaPointerAttributes attributes{};
  CHECK_CUDA_ERROR(cudaPointerGetAttributes(&attributes, ptr.get()));
  EXPECT_EQ(attributes.type, cudaMemoryTypeHost);
  EXPECT_EQ(attributes.hostPointer, ptr.get());
}

TEST_F(CudaUniquePtrTest, MakeUniqueHostArray)
{
  // Test creating an array on host
  auto ptr = autoware::cuda_utils::make_unique_host<float[]>(100, cudaHostAllocDefault);
  EXPECT_NE(ptr.get(), nullptr);
}

TEST_F(CudaUniquePtrTest, DeleterFunctionality)
{
  // Test that CudaDeleter and CudaDeleterHost types exist and are usable
  {
    auto ptr = autoware::cuda_utils::make_unique<int>();
    // Deleter will be called automatically on scope exit
  }

  {
    auto ptr = autoware::cuda_utils::make_unique_host<int>();
    // Deleter will be called automatically on scope exit
  }

  // If we reach here without crashes, deleters worked correctly
  // TODO: Ideally, we would confirm that the memory was freed, but CUDA provides no API for that.
  // Instead, calling any CUDA API on a freed pointer is undefined behavior and usually results in a
  // SEGFAULT.
  SUCCEED();
}
