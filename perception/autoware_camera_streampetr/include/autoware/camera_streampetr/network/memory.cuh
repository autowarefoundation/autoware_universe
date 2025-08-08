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

#include <NvInferRuntime.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

struct Memory
{
  int mem_len;     // Length of the updated memory buffer output from the model (post_memory_length)
  int pre_len;     // Length of the  memory buffer input to the model (pre_memory_length)
  void * mem_buf;  // GPU memory buffer for temporal data storage (size: pre_len)
  float * pre_buf;   // Pointer to pre-memory timestamp buffer [1, pre_len, 1] - stores timestamps
                     // from previous frames
  float * post_buf;  // Pointer to post-memory timestamp buffer [1, mem_len, 1] - stores current
                     // frame timestamps

  cudaStream_t mem_stream;  // CUDA stream for asynchronous memory operations

  void Init(cudaStream_t stream, const int pre_length, const int post_length)
  {
    mem_len = post_length;
    pre_len = pre_length;
    mem_stream = stream;
    cudaMallocAsync(&mem_buf, sizeof(float) * mem_len, mem_stream);
  }

  void StepReset();
  void StepPre(float ts);
  void StepPost(float ts);

  void DebugPrint();
};  // struct Memory
