/*
 * SPDX-FileCopyrightText: Copyright (c) 1993-2025 NVIDIA CORPORATION & AFFILIATES. All rights
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
 *
 */

#include "autoware/select_and_pad_ops/select_and_pad_kernel.h"
#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <cub/cub.cuh>

// Missing CUDA macros
#define CUDA_1D_KERNEL_LOOP(i, n) \
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n); i += blockDim.x * gridDim.x)

#define THREADS_PER_BLOCK 512

#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))

inline int GET_BLOCKS(const int N)
{
  int optimal_block_num = DIVUP(N, THREADS_PER_BLOCK);
  int max_block_num = 4096;
  return std::min(optimal_block_num, max_block_num);
}

// indices: B, P
__global__ void init_select_and_pad(int * indices, int * buf, const int P)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < P) {
    indices[i] = i;
    buf[i] = -1;
  }
}

// feat:    B, Q, C
// indices: B, P
// invalid: C
// out:     B, P, C
// x = indices[b, p]
// out[b, p, :] = feat[b, x, :] if x
template <typename T>
__global__ void select_or_pad(
  T * feat, int * indices, T * invalid, const int B, const int Q, const int C, const int P, T * out)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= B * P) return;

  int b = i / Q;
  int index = indices[i];
  T * curr_f = invalid;
  if (index != -1) {
    // printf("%d, %d\n", i, index);
    curr_f = feat + b * Q * C + index * C;
  }
  T * curr_out = out + b * P * C + i * C;
  for (int c = 0; c < C; c++) {
    curr_out[c] = curr_f[c];
  }
}

// SelectAndPadPlugin::decideTemp implementation moved to plugin file

template <typename T>
void select_and_pad_launch(
  T * feat, int * flags, T * invalid, int B, int Q, int C, int P, T * out, void * tmp,
  size_t tmp_bytes, cudaStream_t stream)
{
  // https://nvidia.github.io/cccl/cub/api/structcub_1_1DeviceSelect.html?highlight=deviceselect#_CPPv4I000EN3cub12DeviceSelect7FlaggedE11cudaError_tPvR6size_t9IteratorT12FlagIterator20NumSelectedIteratorTi12cudaStream_t
  int * buf = reinterpret_cast<int *>(tmp);
  int * indices = reinterpret_cast<int *>(buf + sizeof(int) * Q);
  int * ftmp = reinterpret_cast<int *>(indices + sizeof(int) * Q);
  int * d_num = reinterpret_cast<int *>(ftmp + tmp_bytes);

  init_select_and_pad<<<((Q + 255) / 256), 256, 0, stream>>>(indices, buf, Q);

  // int* h_flags = new int[Q];
  // cudaMemcpy(h_flags, flags, sizeof(int) * Q, cudaMemcpyDeviceToHost);
  // for( int i=0; i<Q; i++ ) {
  //   printf("%d ", h_flags[i]);
  // }
  // printf("\n");
  // print_log("tmp_bytes=%d", tmp_bytes);

  cudaError_t err =
    cub::DeviceSelect::Flagged(ftmp, tmp_bytes, indices, flags, buf, d_num, Q, stream);

  // cudaStreamSynchronize(stream);
  // if( err != cudaSuccess ) {
  //   printf("err = %s\n", cudaGetErrorString(err));
  // }

  // int h_num = 0;
  // cudaMemcpyAsync(&h_num, d_num, sizeof(int), cudaMemcpyDeviceToHost, stream);
  // cudaStreamSynchronize(stream);
  // printf("h_num = %d\n", h_num);

  // int* h_buf = new int[P];
  // cudaMemcpy(h_buf, buf, sizeof(int) * P, cudaMemcpyDeviceToHost);
  // for( int i=0; i<P; i++ ) {
  //   printf("%d ", h_buf[i]);
  // }
  // printf("\n");

  // printf("B=%d, Q=%d, C=%d, P=%d\n", B, Q, C, P);
  select_or_pad<<<((P + 255) / 256), 256, 0, stream>>>(feat, buf, invalid, B, Q, C, P, out);

  // delete[] h_flags;
  // delete[] h_buf;
}

// Launcher function for init_select_and_pad
void init_select_and_pad_launcher(int * indices, int * buf, const int P, cudaStream_t stream)
{
  init_select_and_pad<<<((P + 255) / 256), 256, 0, stream>>>(indices, buf, P);
}

// Template instantiations for select_or_pad_launcher
template void select_and_pad_launch<float>(
  float * feat, int * flags, float * invalid, int B, int Q, int C, int P, float * out, void * tmp,
  size_t tmp_bytes, cudaStream_t stream);

template void select_and_pad_launch<__half>(
  __half * feat, int * flags, __half * invalid, int B, int Q, int C, int P, __half * out,
  void * tmp, size_t tmp_bytes, cudaStream_t stream);
