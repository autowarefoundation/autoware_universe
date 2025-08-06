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

#ifndef AUTOWARE__ROTATE_OPS__ROTATE_KERNEL_H_
#define AUTOWARE__ROTATE_OPS__ROTATE_KERNEL_H_

// #include "cuda_int8.h"
#include <cuda_fp16.h>
#include <cuda_runtime.h>

enum class RotateInterpolation { Bilinear, Nearest };

template <typename T>
void rotate(
  T * output, T * input, T * angle, T * center, int * input_dims, RotateInterpolation interp,
  cudaStream_t stream);

#endif  // AUTOWARE__ROTATE_OPS__ROTATE_KERNEL_H_
