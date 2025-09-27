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

enum class RotateInterpolation { Bilinear, Nearest };

template <typename T>
void rotate(
  T * output, const T * input, const T * angle, const T * center, const int64_t * input_dims,
  RotateInterpolation interp, cudaStream_t stream);

void rotate_h2(
  __half2 * output, const __half2 * input, const __half * angle, const __half * center,
  const int64_t * input_dims, RotateInterpolation interp, cudaStream_t stream);

template <typename T>
void rotate_int8(
  int8_4 * output, float scale_o, const int8_4 * input, float scale_i, const T * angle,
  const T * center, const int64_t * input_dims, RotateInterpolation interp, cudaStream_t stream);
