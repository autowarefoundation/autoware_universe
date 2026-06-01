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

#include "autoware/tensorrt_plugins/spconv_workspace.hpp"

#include <cub/device/device_merge_sort.cuh>
#include <cub/device/device_radix_sort.cuh>

#include <cuda_runtime.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace nvinfer1::plugin
{
namespace
{

template <typename T>
struct Less
{
  __host__ __device__ bool operator()(T const & lhs, T const & rhs) const { return lhs < rhs; }
};

template <typename KeyT>
std::size_t get_radix_sort_pairs_temp_storage_size(int num_items)
{
  if (num_items <= 0) {
    return 0;
  }

  std::size_t temp_storage_bytes = 0;
  KeyT * keys_in = nullptr;
  KeyT * keys_out = nullptr;
  std::int32_t * values_in = nullptr;
  std::int32_t * values_out = nullptr;
  cudaError_t const status = cub::DeviceRadixSort::SortPairs(
    nullptr, temp_storage_bytes, keys_in, keys_out, values_in, values_out, num_items, 0,
    sizeof(KeyT) * 8);
  return status == cudaSuccess ? temp_storage_bytes : 0;
}

template <typename KeyT>
std::size_t get_merge_sort_pairs_temp_storage_size(int num_items)
{
  if (num_items <= 0) {
    return 0;
  }

  std::size_t temp_storage_bytes = 0;
  KeyT * keys = nullptr;
  std::int32_t * values = nullptr;
  cudaError_t const status = cub::DeviceMergeSort::SortPairs(
    nullptr, temp_storage_bytes, keys, values, num_items, Less<KeyT>{});
  return status == cudaSuccess ? temp_storage_bytes : 0;
}

}  // namespace

std::size_t get_spconv_sort_temp_storage_size(
  int num_items, int mask_int_count, bool has_masked_comparator, bool use_int64_keys)
{
  const bool query_64_bit_keys = use_int64_keys || mask_int_count > 1;
  std::size_t temp_storage_bytes =
    query_64_bit_keys ? get_radix_sort_pairs_temp_storage_size<std::uint64_t>(num_items)
                      : get_radix_sort_pairs_temp_storage_size<std::uint32_t>(num_items);

  if (has_masked_comparator) {
    const std::size_t merge_sort_temp_storage_bytes =
      query_64_bit_keys ? get_merge_sort_pairs_temp_storage_size<std::uint64_t>(num_items)
                        : get_merge_sort_pairs_temp_storage_size<std::uint32_t>(num_items);
    temp_storage_bytes = std::max(temp_storage_bytes, merge_sort_temp_storage_bytes);
  }

  return temp_storage_bytes;
}

}  // namespace nvinfer1::plugin
