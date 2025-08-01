// Copyright 2025 The Autoware Contributors
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
//
/*
 * This file includes portions of code directly from the BEVFormer TensorRT implementation
 * by Derry Lin, available at:
 *   https://github.com/DerryHub/BEVFormer_tensorrt
 *
 * The included code is used under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The Autoware Contributors have reused this code as-is in 2025, with no modifications.
 * Original creation by Derry Lin on 2022/10/22.
 */

#pragma once

#include "cuda_int8.h"

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>

template <typename T>
void ms_deformable_im2col_cuda(
  const T * data_value, const int32_t * data_spatial_shapes, const T * data_reference_points,
  const T * data_sampling_offsets, const T * data_attn_weight, const int batch_size,
  const int spatial_size, const int num_heads, const int channels, const int num_levels,
  const int num_query, const int num_point, const int points_per_group, T * data_col,
  cudaStream_t stream);

void ms_deformable_im2col_cuda_h2(
  const __half2 * data_value, const int32_t * data_spatial_shapes,
  const __half2 * data_reference_points, const __half2 * data_sampling_offsets,
  const __half * data_attn_weight, const int batch_size, const int spatial_size,
  const int num_heads, int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, __half2 * data_col, cudaStream_t stream);

template <typename T>
void ms_deformable_im2col_cuda_int8(
  const int8_4 * data_value, float scale_value, const int32_t * data_spatial_shapes,
  const T * data_reference_points, const int8_4 * data_sampling_offsets, float scale_offset,
  const int8_4 * data_attn_weight, float scale_weight, const int batch_size, const int spatial_size,
  const int num_heads, int channels, const int num_levels, const int num_query, const int num_point,
  const int points_per_group, int8_4 * data_col, float scale_out, cudaStream_t stream);
