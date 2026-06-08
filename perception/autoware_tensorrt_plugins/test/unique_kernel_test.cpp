// Copyright 2026 TIER IV, Inc.
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

#include "autoware/unique_ops/unique.hpp"
#include "test_utils.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace
{

using autoware::tensorrt_plugins::test::copy_to_device;
using autoware::tensorrt_plugins::test::copy_to_host;
using autoware::tensorrt_plugins::test::CudaStreamGuard;
using autoware::tensorrt_plugins::test::DeviceBuffer;

struct UniqueReference
{
  std::vector<std::int64_t> unique_values;
  std::vector<std::int64_t> inverse_indices;
  std::vector<std::int64_t> counts;
};

UniqueReference make_unique_reference(const std::vector<std::int64_t> & input)
{
  UniqueReference reference;
  reference.unique_values = input;
  std::sort(reference.unique_values.begin(), reference.unique_values.end());
  reference.unique_values.erase(
    std::unique(reference.unique_values.begin(), reference.unique_values.end()),
    reference.unique_values.end());

  std::map<std::int64_t, std::int64_t> value_to_index;
  for (std::size_t index = 0; index < reference.unique_values.size(); ++index) {
    value_to_index.emplace(reference.unique_values[index], static_cast<std::int64_t>(index));
  }
  reference.counts.resize(reference.unique_values.size(), 0);

  reference.inverse_indices.reserve(input.size());
  for (const auto value : input) {
    const auto index = value_to_index.at(value);
    reference.inverse_indices.push_back(index);
    reference.counts[static_cast<std::size_t>(index)] += 1;
  }

  return reference;
}

TEST(ReferenceKernelsTest, UniqueMatchesCpuReference)
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();

  const std::vector<std::int64_t> input{7, 3, 7, 5, 3, 3, 9, 5, 11, 7};
  const UniqueReference reference = make_unique_reference(input);

  CudaStreamGuard stream;
  DeviceBuffer<std::int64_t> input_d(input.size());
  DeviceBuffer<std::int64_t> unique_d(input.size());
  DeviceBuffer<std::int64_t> inverse_d(input.size());
  DeviceBuffer<std::int64_t> counts_d(input.size());
  DeviceBuffer<std::int64_t> num_unique_d(1U);
  DeviceBuffer<std::uint8_t> workspace_d(get_unique_workspace_size(input.size()));

  copy_to_device(input_d.get(), input);

  ASSERT_EQ(
    unique(
      input_d.get(), unique_d.get(), inverse_d.get(), counts_d.get(), num_unique_d.get(),
      workspace_d.get(), input.size(), get_unique_workspace_size(input.size()), stream.get()),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  const auto num_unique = copy_to_host(num_unique_d.get(), 1U).front();

  const auto unique_values = copy_to_host(unique_d.get(), static_cast<std::size_t>(num_unique));
  const auto inverse_indices = copy_to_host(inverse_d.get(), input.size());
  const auto counts = copy_to_host(counts_d.get(), static_cast<std::size_t>(num_unique));

  EXPECT_EQ(unique_values, reference.unique_values);
  EXPECT_EQ(inverse_indices, reference.inverse_indices);
  EXPECT_EQ(counts, reference.counts);
}

TEST(ReferenceKernelsTest, UniqueEmptyInputWritesZeroCount)
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();

  CudaStreamGuard stream;
  DeviceBuffer<std::int64_t> input_d(1U);
  DeviceBuffer<std::int64_t> unique_d(1U);
  DeviceBuffer<std::int64_t> inverse_d(1U);
  DeviceBuffer<std::int64_t> counts_d(1U);
  DeviceBuffer<std::int64_t> num_unique_d(1U);
  DeviceBuffer<std::uint8_t> workspace_d(1U);

  ASSERT_EQ(
    unique(
      input_d.get(), unique_d.get(), inverse_d.get(), counts_d.get(), num_unique_d.get(),
      workspace_d.get(), 0U, 0U, stream.get()),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  EXPECT_EQ(copy_to_host(num_unique_d.get(), 1U).front(), 0);
}

}  // namespace
