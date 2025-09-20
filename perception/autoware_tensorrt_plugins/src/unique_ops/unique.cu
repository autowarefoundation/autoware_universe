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

#include <thrust/adjacent_difference.h>
#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/scan.h>
#include <thrust/scatter.h>
#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/unique.h>

std::int64_t unique(
  const std::int64_t * input, std::int64_t * unique, std::int64_t * inverse_indices,
  std::int64_t * unique_counts, void * workspace, std::size_t num_input_elements,
  std::size_t unique_workspace_size, cudaStream_t stream)
{
  auto policy = thrust::cuda::par.on(stream);

  thrust::device_ptr<std::int64_t> idx_ptr(reinterpret_cast<std::int64_t *>(workspace));

  thrust::sequence(policy, idx_ptr, idx_ptr + num_input_elements + 1, 0);

  std::int64_t * sorted_input = unique;
  std::int64_t * sorted_idx = thrust::raw_pointer_cast(idx_ptr) + 2 * num_input_elements + 1;
  std::int64_t * inv_loc_ptr = thrust::raw_pointer_cast(idx_ptr) + 3 * num_input_elements + 1;

  void * sort_workspace_ptr =
    reinterpret_cast<void *>(thrust::raw_pointer_cast(idx_ptr) + 4 * num_input_elements + 1);

  auto sort_workspace_size =
    unique_workspace_size - (4 * num_input_elements + 1) * sizeof(std::int64_t);

  cub::DeviceRadixSort::SortPairs(
    sort_workspace_ptr, sort_workspace_size, input, sorted_input, thrust::raw_pointer_cast(idx_ptr),
    sorted_idx, num_input_elements, 0, 64, stream);

  auto equal = [] __device__(const std::int64_t a, const std::int64_t b) { return a == b; };

  auto not_equal = [] __device__(const std::int64_t a, const std::int64_t b) { return a != b; };

  thrust::adjacent_difference(
    policy, sorted_input, sorted_input + num_input_elements, inv_loc_ptr, not_equal);

  cudaMemsetAsync(inv_loc_ptr, 0, sizeof(int64_t), stream);

  thrust::inclusive_scan(policy, inv_loc_ptr, inv_loc_ptr + num_input_elements, inv_loc_ptr);
  thrust::scatter(
    policy, inv_loc_ptr, inv_loc_ptr + num_input_elements, sorted_idx, inverse_indices);

  std::int64_t num_out;

  std::int64_t * range_ptr = idx_ptr.get();
  num_out =
    thrust::unique_by_key(policy, sorted_input, sorted_input + num_input_elements, range_ptr, equal)
      .first -
    sorted_input;

  cudaMemcpyAsync(
    range_ptr + num_out * sizeof(int64_t), &num_input_elements, sizeof(std::int64_t),
    cudaMemcpyHostToDevice, stream);

  thrust::adjacent_difference(policy, range_ptr + 1, range_ptr + num_out + 1, unique_counts);

  return num_out;
}

std::size_t get_unique_workspace_size(std::size_t num_elements)
{
  std::size_t temp_size = 0;
  std::int64_t * int64_nullptr = nullptr;

  cub::DeviceRadixSort::SortPairs(
    nullptr, temp_size, int64_nullptr, int64_nullptr, int64_nullptr, int64_nullptr, num_elements, 0,
    64, nullptr);

  return temp_size + (4 * num_elements + 1) * sizeof(std::int64_t);
}
