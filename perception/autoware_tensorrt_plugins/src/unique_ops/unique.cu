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
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace
{

constexpr int kThreadsPerBlock = 256;

std::size_t align_up(const std::size_t size, const std::size_t alignment)
{
  return ((size + alignment - 1U) / alignment) * alignment;
}

struct UniqueWorkspaceLayout
{
  /// Scratch storage reused by the largest CUB primitive in this implementation.
  void * cub_temp_storage;

  /// Number of bytes reserved for `cub_temp_storage`.
  std::size_t cub_temp_storage_size;

  // One int64 scratch block, then one int32 scratch block:
  //
  // workspace
  // +-----------------------------+ 0
  // | CUB temp storage            | cub_temp_storage_size bytes
  // +-----------------------------+ align_up(cub_temp_storage_size, alignof(int64))
  // | input_positions             | num_input_elements int64
  // | sorted_input                | num_input_elements int64
  // | sorted_input_positions      | num_input_elements int64
  // | unique_offsets_end          | 1 int64
  // | num_unique                  | 1 int32
  // | run_ids                     | num_input_elements int32
  // +-----------------------------+

  /// Original input positions used as values for the initial value/index sort.
  std::int64_t * input_positions;

  /// Sorted copy of the input values.
  std::int64_t * sorted_input;

  /// Original input positions ordered by `sorted_input`; later reused for compact run start
  /// offsets.
  std::int64_t * sorted_input_positions;

  /// Sentinel end offset used to compute the final unique count.
  std::int64_t * unique_offsets_end;

  /// Device scalar storing the number of unique values selected by CUB.
  std::int32_t * num_unique;

  /// Per-sorted-element inclusive run ids used to scatter inverse indices.
  std::int32_t * run_ids;
};

std::size_t query_unique_temp_storage_size(const std::size_t num_elements_in)
{
  std::size_t sort_temp_size = 0;
  std::size_t scan_temp_size = 0;
  std::size_t unique_temp_size = 0;

  std::int64_t * int64_nullptr = nullptr;
  std::int32_t * int32_nullptr = nullptr;

  cub::DeviceRadixSort::SortPairs(
    nullptr, sort_temp_size, int64_nullptr, int64_nullptr, int64_nullptr, int64_nullptr,
    num_elements_in, 0, 64, nullptr);
  cub::DeviceScan::InclusiveSum(
    nullptr, scan_temp_size, int32_nullptr, int32_nullptr, num_elements_in, nullptr);
  cub::DeviceSelect::UniqueByKey(
    nullptr, unique_temp_size, int64_nullptr, int64_nullptr, int64_nullptr, int64_nullptr,
    int32_nullptr, num_elements_in, nullptr);

  return std::max(sort_temp_size, std::max(scan_temp_size, unique_temp_size));
}

UniqueWorkspaceLayout make_unique_workspace_layout(
  void * workspace_inout, const std::size_t num_input_elements_in,
  const std::size_t cub_temp_storage_size_in)
{
  const auto scratch_offset = align_up(cub_temp_storage_size_in, alignof(std::int64_t));
  auto * scratch = reinterpret_cast<char *>(workspace_inout) + scratch_offset;

  auto * input_positions = reinterpret_cast<std::int64_t *>(scratch);
  auto * sorted_input = input_positions + num_input_elements_in;
  auto * sorted_input_positions = sorted_input + num_input_elements_in;
  auto * unique_offsets_end = sorted_input_positions + num_input_elements_in;
  auto * num_unique = reinterpret_cast<std::int32_t *>(unique_offsets_end + 1U);
  auto * run_ids = reinterpret_cast<std::int32_t *>(num_unique + 1U);

  return UniqueWorkspaceLayout{
    workspace_inout,        cub_temp_storage_size_in, input_positions, sorted_input,
    sorted_input_positions, unique_offsets_end,       num_unique,      run_ids};
}

__global__ void mark_run_starts(
  const std::int64_t * sorted_input_in, std::int32_t * run_ids_out,
  const std::size_t num_input_elements_in)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= num_input_elements_in) {
    return;
  }

  run_ids_out[index] =
    (index == 0U || sorted_input_in[index] != sorted_input_in[index - 1U]) ? 1 : 0;
}

__global__ void fill_iota(std::int64_t * output_out, const std::size_t num_input_elements_in)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= num_input_elements_in) {
    return;
  }

  output_out[index] = static_cast<std::int64_t>(index);
}

__global__ void scatter_inverse_indices(
  const std::int64_t * sorted_idx_in, const std::int32_t * run_ids_in,
  std::int64_t * inverse_indices_out, const std::size_t num_input_elements_in)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= num_input_elements_in) {
    return;
  }

  inverse_indices_out[sorted_idx_in[index]] = static_cast<std::int64_t>(run_ids_in[index] - 1);
}

__global__ void write_unique_offset_sentinel(
  std::int64_t * unique_offsets_end_out, const std::size_t num_input_elements_in)
{
  *unique_offsets_end_out = static_cast<std::int64_t>(num_input_elements_in);
}

__global__ void write_unique_counts(
  const std::int64_t * unique_offsets_in, const std::int32_t * num_unique_in,
  const std::int64_t * unique_offsets_end_in, std::int64_t * unique_counts_out)
{
  const auto index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= static_cast<std::size_t>(*num_unique_in)) {
    return;
  }

  const auto next_offset = (index + 1U == static_cast<std::size_t>(*num_unique_in))
                             ? *unique_offsets_end_in
                             : unique_offsets_in[index + 1U];
  unique_counts_out[index] = next_offset - unique_offsets_in[index];
}

}  // namespace

std::int64_t unique(
  const std::int64_t * input_in, std::int64_t * unique_values_out,
  std::int64_t * inverse_indices_out, std::int64_t * unique_counts_out, void * workspace_inout,
  std::size_t num_input_elements_in, std::size_t workspace_size_in, cudaStream_t stream_in)
{
  if (num_input_elements_in == 0U) {
    return 0;
  }

  assert(workspace_size_in >= get_unique_workspace_size(num_input_elements_in));
  (void)workspace_size_in;

  const auto cub_temp_storage_size = get_unique_temp_storage_size(num_input_elements_in);
  auto layout =
    make_unique_workspace_layout(workspace_inout, num_input_elements_in, cub_temp_storage_size);

  const auto num_blocks =
    static_cast<unsigned int>((num_input_elements_in + kThreadsPerBlock - 1U) / kThreadsPerBlock);

  // 1. Sort values while carrying their original input positions.
  fill_iota<<<num_blocks, kThreadsPerBlock, 0, stream_in>>>(
    layout.input_positions, num_input_elements_in);
  cub::DeviceRadixSort::SortPairs(
    layout.cub_temp_storage, layout.cub_temp_storage_size, input_in, layout.sorted_input,
    layout.input_positions, layout.sorted_input_positions, num_input_elements_in, 0, 64, stream_in);

  // 2. Convert sorted run starts into sorted-position -> unique-index ids.
  mark_run_starts<<<num_blocks, kThreadsPerBlock, 0, stream_in>>>(
    layout.sorted_input, layout.run_ids, num_input_elements_in);
  cub::DeviceScan::InclusiveSum(
    layout.cub_temp_storage, layout.cub_temp_storage_size, layout.run_ids, layout.run_ids,
    num_input_elements_in, stream_in);

  // 3. Scatter unique ids back to original input order.
  scatter_inverse_indices<<<num_blocks, kThreadsPerBlock, 0, stream_in>>>(
    layout.sorted_input_positions, layout.run_ids, inverse_indices_out, num_input_elements_in);

  // 4. Compact sorted runs into unique values and each run's start offset.
  auto * unique_offsets_inout = layout.sorted_input_positions;
  cub::DeviceSelect::UniqueByKey(
    layout.cub_temp_storage, layout.cub_temp_storage_size, layout.sorted_input,
    layout.input_positions, unique_values_out, unique_offsets_inout, layout.num_unique,
    num_input_elements_in, stream_in);

  // 5. Turn run start offsets into counts.
  // CUB writes each run's start offset, but not the final end offset. Store the sentinel in the
  // fixed extra int64 slot so the final count can use the input length as its end boundary without
  // overwriting compact offsets or the selected-count scalar.
  write_unique_offset_sentinel<<<1, 1, 0, stream_in>>>(
    layout.unique_offsets_end, num_input_elements_in);
  write_unique_counts<<<num_blocks, kThreadsPerBlock, 0, stream_in>>>(
    unique_offsets_inout, layout.num_unique, layout.unique_offsets_end, unique_counts_out);

  std::int32_t num_out = 0;
  cudaMemcpyAsync(
    &num_out, layout.num_unique, sizeof(std::int32_t), cudaMemcpyDeviceToHost, stream_in);
  cudaStreamSynchronize(stream_in);
  return static_cast<std::int64_t>(num_out);
}

std::size_t get_unique_temp_storage_size(std::size_t num_elements_in)
{
  return query_unique_temp_storage_size(num_elements_in);
}

std::size_t get_unique_workspace_size(std::size_t num_elements_in)
{
  const auto temp_size = query_unique_temp_storage_size(num_elements_in);
  const auto scratch_offset = align_up(temp_size, alignof(std::int64_t));
  return scratch_offset + (3 * num_elements_in + 1U) * sizeof(std::int64_t) +
         (num_elements_in + 1U) * sizeof(std::int32_t);
}
