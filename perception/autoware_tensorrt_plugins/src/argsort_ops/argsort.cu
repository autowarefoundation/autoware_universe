// Copyright 2025 TIER IV, Inc.
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
#include "autoware/tensorrt_plugins/kernel_utils.hpp"

#include <cub/cub.cuh>

#include <cstddef>

namespace
{

constexpr int kThreadsPerBlock = 256;

}  // namespace

cudaError_t argsort(
  const std::int64_t * input_d, std::int64_t * output_d, void * workspace, std::size_t num_elements,
  std::size_t argsort_workspace_size, cudaStream_t stream)
{
  if (num_elements == 0U) {
    return cudaSuccess;
  }

  const auto scratch_offset =
    autoware::tensorrt_plugins::align_up(argsort_workspace_size, alignof(std::int64_t));
  auto * input_idx_d =
    reinterpret_cast<std::int64_t *>(reinterpret_cast<std::byte *>(workspace) + scratch_offset);
  auto * input_sorted_d = input_idx_d + num_elements;

  const auto num_blocks =
    static_cast<unsigned int>((num_elements + kThreadsPerBlock - 1U) / kThreadsPerBlock);
  autoware::tensorrt_plugins::fill_iota<<<num_blocks, kThreadsPerBlock, 0, stream>>>(
    input_idx_d, num_elements);
  if (const auto status = cudaPeekAtLastError(); status != cudaSuccess) {
    return status;
  }

  return cub::DeviceRadixSort::SortPairs(
    workspace, argsort_workspace_size, input_d, input_sorted_d, input_idx_d, output_d, num_elements,
    0, 64, stream);
}

std::size_t get_argsort_workspace_size(std::size_t num_elements)
{
  std::size_t temp_size = 0;

  std::int64_t * int64_nullptr = nullptr;

  cub::DeviceRadixSort::SortPairs(
    nullptr, temp_size, int64_nullptr, int64_nullptr, int64_nullptr, int64_nullptr, num_elements, 0,
    64, nullptr);

  return temp_size;
}
