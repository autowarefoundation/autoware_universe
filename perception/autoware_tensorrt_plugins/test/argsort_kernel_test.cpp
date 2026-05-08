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

#include "autoware/argsort_ops/argsort.hpp"
#include "test_utils.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace
{

using autoware::tensorrt_plugins::test::copy_to_device;
using autoware::tensorrt_plugins::test::copy_to_host;
using autoware::tensorrt_plugins::test::CudaStreamGuard;
using autoware::tensorrt_plugins::test::DeviceBuffer;

std::size_t get_argsort_total_workspace_size(const std::size_t num_elements)
{
  const auto temp_size = get_argsort_workspace_size(num_elements);
  const auto scratch_offset =
    ((temp_size + alignof(std::int64_t) - 1U) / alignof(std::int64_t)) * alignof(std::int64_t);
  return scratch_offset + sizeof(std::int64_t) * 2U * num_elements;
}

std::vector<std::int64_t> make_argsort_reference(const std::vector<std::int64_t> & input)
{
  std::vector<std::int64_t> indices(input.size());
  for (std::size_t index = 0; index < input.size(); ++index) {
    indices[index] = static_cast<std::int64_t>(index);
  }

  std::stable_sort(
    indices.begin(), indices.end(),
    [&input](const std::int64_t lhs, const std::int64_t rhs) { return input[lhs] < input[rhs]; });

  return indices;
}

TEST(ReferenceKernelsTest, ArgsortMatchesCpuReference)
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();

  const std::vector<std::int64_t> input{7, 3, 7, 5, 3, 3, 9, 5, 11, 7};
  const auto reference = make_argsort_reference(input);

  CudaStreamGuard stream;
  DeviceBuffer<std::int64_t> input_d(input.size());
  DeviceBuffer<std::int64_t> output_d(input.size());
  DeviceBuffer<std::uint8_t> workspace_d(get_argsort_total_workspace_size(input.size()));

  copy_to_device(input_d.get(), input);

  ASSERT_EQ(
    argsort(
      input_d.get(), output_d.get(), workspace_d.get(), input.size(),
      get_argsort_workspace_size(input.size()), stream.get()),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  EXPECT_EQ(copy_to_host(output_d.get(), input.size()), reference);
}

}  // namespace
