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

#ifndef AUTOWARE__UNIQUE_OPS__UNIQUE_HPP_
#define AUTOWARE__UNIQUE_OPS__UNIQUE_HPP_

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>

/// Computes sorted unique int64 values, inverse indices, counts, and the number of uniques on
/// `stream_in`.
///
/// Input contract:
/// - `input_in` points to `num_input_elements_in` int64 values.
/// - `unique_values_out`, `inverse_indices_out`, and `unique_counts_out` each point to at least
///   `num_input_elements_in` int64 values. The true `unique_counts_out` length is written to
///   `num_unique_elements_out`, but the buffer must allow the worst case where every input is
///   unique.
/// - `num_unique_elements_out` points to one int64 device scalar.
/// - `workspace_inout` points to at least `workspace_size_in` bytes.
/// - `workspace_size_in` must be no smaller than
///   `get_unique_workspace_size(num_input_elements_in)`.
/// - All pointers are device pointers.
///
/// Output contract:
/// - Returns `cudaSuccess` if all work was enqueued successfully; otherwise returns the first CUDA
///   error encountered.
/// - `unique_values_out[0:*num_unique_elements_out]` contains the input values in ascending order,
///   with duplicates removed.
/// - `inverse_indices_out[0:num_input_elements_in]` maps each original input element to its
///   `unique_values_out` index.
/// - `unique_counts_out[0:*num_unique_elements_out]` contains the occurrence count for each unique
///   value.
/// - `*num_unique_elements_out` contains the number of unique values found.
cudaError_t unique(
  const std::int64_t * input_in, std::int64_t * unique_values_out,
  std::int64_t * inverse_indices_out, std::int64_t * unique_counts_out,
  std::int64_t * num_unique_elements_out, void * workspace_inout, std::size_t num_input_elements_in,
  std::size_t workspace_size_in, cudaStream_t stream_in);

/// Bytes of CUB temporary storage required by the largest CUB primitive used by `unique`.
std::size_t get_unique_temp_storage_size(std::size_t num_elements_in);

/// Total TensorRT workspace bytes required by `unique`, including CUB temporary storage and
/// all variant-local scratch arrays.
std::size_t get_unique_workspace_size(std::size_t num_elements_in);

#endif  // AUTOWARE__UNIQUE_OPS__UNIQUE_HPP_
