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

#include "autoware/camera_streampetr/network/memory.cuh"

#include <stdio.h>

__global__ void ApplyDeltaFromMem(float delta, float * mem, float * buf, int n_elem)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < n_elem) {
    float v = mem[idx] + delta;
    buf[idx] = v;
  }
}

__global__ void ApplyDeltaToMem(float delta, float * mem, float * buf, int n_elem)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < n_elem) {
    float v = buf[idx];
    mem[idx] = v - delta;
  }
}

void Memory::StepReset()
{
  if (mem_stream == nullptr) {
    fprintf(stderr, "Memory stream is not initialized.\n");
    return;
  }
  // reset pre_buf to zero
  cudaMemsetAsync(pre_buf, 0, sizeof(float) * mem_len, mem_stream);
}

void Memory::StepPre(float ts)
{
  if (mem_stream == nullptr) {
    fprintf(stderr, "Memory stream is not initialized.\n");
    return;
  }
  // update timestamp in pre_update_memory
  ApplyDeltaFromMem<<<pre_len, 1, 0, mem_stream>>>(
    ts, reinterpret_cast<float *>(mem_buf), pre_buf, pre_len);
}

void Memory::StepPost(float ts)
{
  if (mem_stream == nullptr) {
    fprintf(stderr, "Memory stream is not initialized.\n");
    return;
  }
  // update timestamp in post_update_memory
  ApplyDeltaToMem<<<mem_len, 1, 0, mem_stream>>>(
    ts, reinterpret_cast<float *>(mem_buf), post_buf, mem_len);
}

void Memory::DebugPrint()
{
  if (mem_stream == nullptr) {
    fprintf(stderr, "Memory stream is not initialized.\n");
    return;
  }
  float temp_buf[16];
  cudaMemcpyAsync(
    reinterpret_cast<void *>(temp_buf), mem_buf, sizeof(float) * 16, cudaMemcpyDeviceToHost,
    mem_stream);
  for (int i = 0; i < 16; i++) {
    printf("%f ", temp_buf[i]);
  }
}
