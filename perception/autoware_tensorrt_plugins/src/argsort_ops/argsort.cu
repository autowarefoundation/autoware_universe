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

#include <cub/cub.cuh>

#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/sequence.h>

cudaError_t argsort(
  const std::int64_t * input_d, std::int64_t * output_d, void * workspace, std::size_t num_elements,
  std::size_t argsort_workspace_size, cudaStream_t stream)
{
  int workspace_offset = (argsort_workspace_size + sizeof(std::int64_t) - 1) / sizeof(std::int64_t);
  thrust::device_ptr<std::int64_t> idx_ptr(
    &reinterpret_cast<std::int64_t *>(workspace)[workspace_offset]);

  thrust::sequence(thrust::cuda::par.on(stream), idx_ptr, idx_ptr + num_elements, 0);

  std::int64_t * input_sorted_d = thrust::raw_pointer_cast(idx_ptr) + num_elements;

  return cub::DeviceRadixSort::SortPairs(
    workspace, argsort_workspace_size, input_d, input_sorted_d, thrust::raw_pointer_cast(idx_ptr),
    output_d, num_elements, 0, 64, stream);
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
