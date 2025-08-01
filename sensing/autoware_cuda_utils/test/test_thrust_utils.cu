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

#include "autoware/cuda_utils/cuda_gtest_utils.hpp"
#include "autoware/cuda_utils/stream_unique_ptr.hpp"

#include <autoware/cuda_utils/thrust_utils.hpp>

#include <gtest/gtest.h>
#include <thrust/logical.h>

class ThrustUtilsTest : public autoware::cuda_utils::CudaTest
{
};

TEST_F(ThrustUtilsTest, ThrustOnStreamFunction)
{
  // Test that thrust_on_stream returns a valid execution policy
  cudaStream_t stream{};
  cudaStreamCreate(&stream);

  auto policy = autoware::cuda_utils::thrust_on_stream(stream);
  // If this compiles and runs without error, the function works
  (void)policy;  // Suppress unused variable warning
  SUCCEED();

  cudaStreamDestroy(stream);
}

TEST_F(ThrustUtilsTest, FillFunctionality)
{
  auto stream = autoware::cuda_utils::makeCudaStream();

  thrust::device_vector<int> vec(100, 0);
  autoware::cuda_utils::thrust_stream::fill(vec, 42, *stream);

  // Synchronize to ensure the operation completed
  cudaStreamSynchronize(*stream);

  // Check if all elements are filled with 42
  EXPECT_EQ(thrust::count(vec.begin(), vec.end(), 42), 100);
}
TEST_F(ThrustUtilsTest, FillNFunctionality)
{
  auto stream = autoware::cuda_utils::makeCudaStream();

  thrust::device_vector<int> vec(100);
  thrust::fill(vec.begin(), vec.end(), 0);  // Initialize with zeros

  autoware::cuda_utils::thrust_stream::fill_n(vec, 50, 99, *stream);

  // Synchronize to ensure the operation completed
  cudaStreamSynchronize(*stream);

  // Check if the first 50 elements are filled with 99
  EXPECT_EQ(thrust::count(vec.begin(), vec.begin() + 50, 99), 50);
  // Check if the rest are still zero
  EXPECT_EQ(thrust::count(vec.begin() + 50, vec.end(), 0), 50);
}

TEST_F(ThrustUtilsTest, CountFunctionality)
{
  auto stream = autoware::cuda_utils::makeCudaStream();

  thrust::device_vector<int> vec(20);
  thrust::fill(vec.begin(), vec.end(), 0);
  thrust::fill(vec.begin(), vec.begin() + 3, 1);  // Three '1's at the beginning
  thrust::fill(vec.end() - 3, vec.end(), 1);      // Three '1's at the end

  long count = autoware::cuda_utils::thrust_stream::count(vec, 1, *stream);

  // Synchronize to ensure the operation completed
  cudaStreamSynchronize(*stream);

  EXPECT_EQ(count, 6);  // There are six '1's in the vector
}
