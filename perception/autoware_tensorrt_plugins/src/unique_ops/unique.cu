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

// From PyTorch:
//
// Copyright (c) 2016-     Facebook, Inc            (Adam Paszke)
// Copyright (c) 2014-     Facebook, Inc            (Soumith Chintala)
// Copyright (c) 2011-2014 Idiap Research Institute (Ronan Collobert)
// Copyright (c) 2012-2014 Deepmind Technologies    (Koray Kavukcuoglu)
// Copyright (c) 2011-2012 NEC Laboratories America (Koray Kavukcuoglu)
// Copyright (c) 2011-2013 NYU                      (Clement Farabet)
// Copyright (c) 2006-2010 NEC Laboratories America (Ronan Collobert, Leon Bottou, Iain Melvin,
// Jason Weston) Copyright (c) 2006      Idiap Research Institute (Samy Bengio) Copyright (c)
// 2001-2004 Idiap Research Institute (Ronan Collobert, Samy Bengio, Johnny Mariethoz)
//
// From Caffe2:
//
// Copyright (c) 2016-present, Facebook Inc. All rights reserved.
//
// All contributions by Facebook:
// Copyright (c) 2016 Facebook Inc.
//
// All contributions by Google:
// Copyright (c) 2015 Google Inc.
// All rights reserved.
//
// All contributions by Yangqing Jia:
// Copyright (c) 2015 Yangqing Jia
// All rights reserved.
//
// All contributions by Kakao Brain:
// Copyright 2019-2020 Kakao Brain
//
// All contributions by Cruise LLC:
// Copyright (c) 2022 Cruise LLC.
// All rights reserved.
//
// All contributions by Tri Dao:
// Copyright (c) 2024 Tri Dao.
// All rights reserved.
//
// All contributions by Arm:
// Copyright (c) 2021, 2023-2024 Arm Limited and/or its affiliates
//
// All contributions from Caffe:
// Copyright(c) 2013, 2014, 2015, the respective contributors
// All rights reserved.
//
// All other contributions:
// Copyright(c) 2015, 2016 the respective contributors
// All rights reserved.
//
// Caffe2 uses a copyright model similar to Caffe: each contributor holds
// copyright over their contributions to Caffe2. The project versioning records
// all such contribution and copyright details. If a contributor wants to further
// mark their specific copyright on a particular contribution, they should
// indicate their copyright solely in the commit message of the change when it is
// committed.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// 3. Neither the names of Facebook, Deepmind Technologies, NYU, NEC Laboratories America
//    and IDIAP Research Institute nor the names of its contributors may be
//    used to endorse or promote products derived from this software without
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "autoware/unique_ops/unique.hpp"

#include <cub/cub.cuh>

#include <algorithm>
#include <cstdint>

namespace
{

constexpr int kThreadsPerBlock = 256;

std::size_t align_up(const std::size_t size, const std::size_t alignment)
{
  return ((size + alignment - 1U) / alignment) * alignment;
}

std::size_t query_unique_temp_storage_size(const std::size_t num_elements)
{
  std::size_t sort_temp_size = 0;
  std::size_t scan_temp_size = 0;
  std::size_t unique_temp_size = 0;

  std::int64_t * int64_nullptr = nullptr;
  std::int32_t * int32_nullptr = nullptr;

  cub::DeviceRadixSort::SortPairs(
    nullptr, sort_temp_size, int64_nullptr, int64_nullptr, int64_nullptr, int64_nullptr,
    num_elements, 0, 64, nullptr);
  cub::DeviceScan::InclusiveSum(
    nullptr, scan_temp_size, int32_nullptr, int32_nullptr, num_elements, nullptr);
  cub::DeviceSelect::UniqueByKey(
    nullptr, unique_temp_size, int64_nullptr, int64_nullptr, int64_nullptr, int64_nullptr,
    int64_nullptr, num_elements, nullptr);

  return std::max(sort_temp_size, std::max(scan_temp_size, unique_temp_size));
}

__global__ void mark_run_starts(
  const std::int64_t * sorted_input, std::int32_t * run_ids, const std::size_t num_input_elements)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= num_input_elements) {
    return;
  }

  run_ids[index] = (index == 0U || sorted_input[index] != sorted_input[index - 1U]) ? 1 : 0;
}

__global__ void fill_iota(std::int64_t * output, const std::size_t num_input_elements)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= num_input_elements) {
    return;
  }

  output[index] = static_cast<std::int64_t>(index);
}

__global__ void scatter_inverse_indices(
  const std::int64_t * sorted_idx, const std::int32_t * run_ids, std::int64_t * inverse_indices,
  const std::size_t num_input_elements)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= num_input_elements) {
    return;
  }

  inverse_indices[sorted_idx[index]] = static_cast<std::int64_t>(run_ids[index] - 1);
}

__global__ void write_unique_offset_sentinel(
  std::int64_t * unique_offsets, const std::int64_t * num_unique,
  const std::size_t num_input_elements)
{
  unique_offsets[*num_unique] = static_cast<std::int64_t>(num_input_elements);
}

__global__ void write_unique_counts(
  const std::int64_t * unique_offsets, const std::int64_t * num_unique,
  std::int64_t * unique_counts)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= static_cast<std::size_t>(*num_unique)) {
    return;
  }

  unique_counts[index] = unique_offsets[index + 1U] - unique_offsets[index];
}

}  // namespace

std::int64_t unique(
  const std::int64_t * input, std::int64_t * unique, std::int64_t * inverse_indices,
  std::int64_t * unique_counts, void * workspace, std::size_t num_input_elements,
  std::size_t unique_workspace_size, cudaStream_t stream)
{
  if (num_input_elements == 0U) {
    return 0;
  }

  const auto temp_storage_size = get_unique_temp_storage_size(num_input_elements);
  const auto scratch_offset = align_up(temp_storage_size, alignof(std::int64_t));
  auto * scratch = reinterpret_cast<char *>(workspace) + scratch_offset;

  auto * input_positions = reinterpret_cast<std::int64_t *>(scratch);
  auto * sorted_input = input_positions + num_input_elements;
  auto * unique_offsets = sorted_input + num_input_elements;
  auto * num_unique_d = unique_offsets + num_input_elements + 1U;
  auto * run_ids = reinterpret_cast<std::int32_t *>(num_unique_d + 1U);

  const auto num_blocks =
    static_cast<unsigned int>((num_input_elements + kThreadsPerBlock - 1U) / kThreadsPerBlock);

  fill_iota<<<num_blocks, kThreadsPerBlock, 0, stream>>>(input_positions, num_input_elements);
  cub::DeviceRadixSort::SortPairs(
    workspace, temp_storage_size, input, sorted_input, input_positions, unique_offsets,
    num_input_elements, 0, 64, stream);

  mark_run_starts<<<num_blocks, kThreadsPerBlock, 0, stream>>>(
    sorted_input, run_ids, num_input_elements);
  cub::DeviceScan::InclusiveSum(
    workspace, temp_storage_size, run_ids, run_ids, num_input_elements, stream);

  scatter_inverse_indices<<<num_blocks, kThreadsPerBlock, 0, stream>>>(
    unique_offsets, run_ids, inverse_indices, num_input_elements);

  cub::DeviceSelect::UniqueByKey(
    workspace, temp_storage_size, sorted_input, input_positions, unique, unique_offsets,
    num_unique_d, num_input_elements, stream);

  write_unique_offset_sentinel<<<1, 1, 0, stream>>>(
    unique_offsets, num_unique_d, num_input_elements);
  write_unique_counts<<<num_blocks, kThreadsPerBlock, 0, stream>>>(
    unique_offsets, num_unique_d, unique_counts);

  std::int64_t num_out = 0;
  cudaMemcpyAsync(&num_out, num_unique_d, sizeof(std::int64_t), cudaMemcpyDeviceToHost, stream);
  cudaStreamSynchronize(stream);
  return num_out;
}

std::size_t get_unique_temp_storage_size(std::size_t num_elements)
{
  return query_unique_temp_storage_size(num_elements);
}

std::size_t get_unique_workspace_size(std::size_t num_elements)
{
  const auto temp_size = query_unique_temp_storage_size(num_elements);
  const auto scratch_offset = align_up(temp_size, alignof(std::int64_t));
  return scratch_offset + (3 * num_elements + 2U) * sizeof(std::int64_t) +
         num_elements * sizeof(std::int32_t);
}
