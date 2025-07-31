// Copyright 2025 The Autoware Contributors

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

/*
 * Portions of this code are derived from the BEVFormer TensorRT implementation by Derry Lin:
 * https://github.com/DerryHub/BEVFormer_tensorrt
 *
 * Original code licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * Modified by The Autoware Contributors in 2025.
 * Original creation by Derry Lin on 2022/10/22.
 */

#pragma once

#include "cuda_int8.h"

#include <cublas_v2.h>

template <typename T>
void ModulatedDeformConvForwardCUDAKernel(
  const T * input, const T * weight, const T * bias, const T * offset, const T * mask, T * output,
  void * workspace, int batch, int channels, int height, int width, int channels_out, int kernel_w,
  int kernel_h, int stride_w, int stride_h, int pad_w, int pad_h, int dilation_w, int dilation_h,
  int group, int deformable_group, int im2col_step, cublasHandle_t cublas_handle,
  cudaStream_t stream);

template <typename T>
void ModulatedDeformConvForwardCUDAKernel_int8(
  const int8_4 * input, const float & scale_i, const int8_4 * weight, const float & scale_w,
  const T * bias, const int8_t * offset, const float & scale_off, const int8_t * mask,
  const float & scale_mask, int8_t * output, const float & scale_o, void * workspace, int batch,
  int channels, int height, int width, int channels_out, int kernel_w, int kernel_h, int stride_w,
  int stride_h, int pad_w, int pad_h, int dilation_w, int dilation_h, int group,
  int deformable_group, int im2col_step, cublasHandle_t cublas_handle, cudaStream_t stream);
