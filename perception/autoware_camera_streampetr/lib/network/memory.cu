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

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <stdio.h>

#include <stdexcept>

namespace
{
constexpr int THREADS_PER_BLOCK = 256;

int blocks_for(const int n_elem)
{
  return (n_elem + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
}

// A null stream means the caller skipped init(); throw rather than silently leave stale state.
void require_stream(const cudaStream_t stream, const char * where)
{
  if (stream == nullptr) {
    throw std::runtime_error(std::string("Memory::") + where + ": init() was never called.");
  }
}
}  // namespace

__global__ void apply_delta_from_mem(float delta, float * mem, float * buf, int n_elem)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < n_elem) {
    float v = mem[idx] + delta;
    buf[idx] = v;
  }
}

__global__ void apply_delta_to_mem(float delta, float * mem, float * buf, int n_elem)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < n_elem) {
    float v = buf[idx];
    mem[idx] = v - delta;
  }
}

void Memory::init(cudaStream_t stream, const int pre_length, const int post_length)
{
  mem_len = post_length;
  pre_len = pre_length;
  mem_stream = stream;
  CHECK_CUDA_ERROR(cudaMallocAsync(&mem_buf, sizeof(float) * mem_len, mem_stream));
  clear();
}

void Memory::release()
{
  if (mem_buf != nullptr) {
    cudaFreeAsync(mem_buf, mem_stream);
    mem_buf = nullptr;
  }
}

void Memory::clear()
{
  require_stream(mem_stream, "Clear");
  CHECK_CUDA_ERROR(cudaMemsetAsync(mem_buf, 0, sizeof(float) * mem_len, mem_stream));
}

void Memory::step_reset()
{
  require_stream(mem_stream, "step_reset");
  // pre_buf is pre_len long; mem_len (post_memory_length) is larger and would write past it.
  CHECK_CUDA_ERROR(cudaMemsetAsync(pre_buf, 0, sizeof(float) * pre_len, mem_stream));
}

void Memory::step_pre(float ts)
{
  require_stream(mem_stream, "step_pre");
  // update timestamp in pre_update_memory
  apply_delta_from_mem<<<blocks_for(pre_len), THREADS_PER_BLOCK, 0, mem_stream>>>(
    ts, reinterpret_cast<float *>(mem_buf), pre_buf, pre_len);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

void Memory::step_post(float ts)
{
  require_stream(mem_stream, "step_post");
  // update timestamp in post_update_memory
  apply_delta_to_mem<<<blocks_for(mem_len), THREADS_PER_BLOCK, 0, mem_stream>>>(
    ts, reinterpret_cast<float *>(mem_buf), post_buf, mem_len);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

void Memory::debug_print()
{
  require_stream(mem_stream, "debug_print");
  float temp_buf[16];
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    reinterpret_cast<void *>(temp_buf), mem_buf, sizeof(float) * 16, cudaMemcpyDeviceToHost,
    mem_stream));
  // The copy is asynchronous, so temp_buf is only valid once the stream has drained.
  CHECK_CUDA_ERROR(cudaStreamSynchronize(mem_stream));
  for (int i = 0; i < 16; i++) {
    printf("%f ", temp_buf[i]);
  }
}
