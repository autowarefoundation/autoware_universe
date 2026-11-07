// Copyright 2025 TIER IV
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

/*
 * SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights
 * reserved. SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTOWARE__CAMERA_STREAMPETR__NETWORK__MEMORY_CUH_
#define AUTOWARE__CAMERA_STREAMPETR__NETWORK__MEMORY_CUH_

#include <NvInferRuntime.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

struct Memory
{
  int mem_len = 0;             // post_memory_length
  int pre_len = 0;             // pre_memory_length
  void * mem_buf = nullptr;    // temporal storage (size: mem_len)
  float * pre_buf = nullptr;   // pre-memory timestamps [1, pre_len, 1]
  float * post_buf = nullptr;  // post-memory timestamps [1, mem_len, 1]

  cudaStream_t mem_stream = nullptr;

  // Allocates and zeroes mem_buf; step_pre reads it on the very first frame.
  void init(cudaStream_t stream, const int pre_length, const int post_length);
  void release();

  // Zeroes mem_buf; use whenever the temporal state has to be dropped.
  void clear();

  void step_reset();
  void step_pre(float ts);
  void step_post(float ts);

  void debug_print();
};  // struct Memory

#endif  // AUTOWARE__CAMERA_STREAMPETR__NETWORK__MEMORY_CUH_
