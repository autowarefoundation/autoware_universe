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

#ifndef AUTOWARE__SELECT_AND_PAD_OPS__SELECT_AND_PAD_KERNEL_H_
#define AUTOWARE__SELECT_AND_PAD_OPS__SELECT_AND_PAD_KERNEL_H_

#include <cuda_fp16.h>
#include <cuda_runtime.h>

// Forward declarations of kernel functions
void init_select_and_pad_launcher(int * indices, int * buf, const int P, cudaStream_t stream);

template <typename T>
void select_or_pad_launcher(
  T * feat, int * indices, T * invalid, const int B, const int Q, const int C, const int P, T * out,
  cudaStream_t stream);

#endif  // AUTOWARE__SELECT_AND_PAD_OPS__SELECT_AND_PAD_KERNEL_H_
