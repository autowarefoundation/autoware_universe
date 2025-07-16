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

#include <autoware/cuda_utils/stream_unique_ptr.hpp>

#include <gtest/gtest.h>

TEST(StreamUniquePtrTest, MakeCudaStreamDefault)
{
  // Test creating a CUDA stream with default flags
  auto stream = autoware::cuda_utils::makeCudaStream();
  EXPECT_NE(stream.get(), nullptr);

  // Check that the stream is valid
  cudaStream_t raw_stream = *stream.get();
  EXPECT_EQ(cudaStreamQuery(raw_stream), cudaSuccess);
}

TEST(StreamUniquePtrTest, MakeCudaStreamWithFlags)
{
  // Test creating a CUDA stream with custom flags
  auto stream = autoware::cuda_utils::makeCudaStream(cudaStreamNonBlocking);
  EXPECT_NE(stream.get(), nullptr);

  // Check that the stream is valid
  cudaStream_t raw_stream = *stream.get();
  EXPECT_EQ(cudaStreamQuery(raw_stream), cudaSuccess);

  // Check that the stream is non-blocking
  unsigned int flags{};
  CHECK_CUDA_ERROR(cudaStreamGetFlags(raw_stream, &flags));
  EXPECT_EQ(flags, cudaStreamNonBlocking);
}

TEST(StreamUniquePtrTest, StreamDeleterFunctionality)
{
  // Test that StreamDeleter properly handles cleanup
  {
    auto stream = autoware::cuda_utils::makeCudaStream();
    // Deleter will be called automatically on scope exit
  }
}

TEST(StreamUniquePtrTest, StreamReset)
{
  // Test that we can reset a stream
  auto stream = autoware::cuda_utils::makeCudaStream();
  EXPECT_NE(stream.get(), nullptr);
  stream.reset();
  EXPECT_EQ(stream.get(), nullptr);
}

TEST(StreamUniquePtrTest, StreamMove)
{
  // Test moving a stream unique pointer
  auto stream1 = autoware::cuda_utils::makeCudaStream();
  EXPECT_NE(stream1.get(), nullptr);

  auto stream2 = std::move(stream1);
  EXPECT_EQ(stream1.get(), nullptr);
  EXPECT_NE(stream2.get(), nullptr);

  // Check that the moved stream is still valid
  EXPECT_EQ(cudaStreamQuery(*stream2), cudaSuccess);
}
