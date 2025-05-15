// Copyright (c) 2020 Matthias Fey <matthias.fey@tu-dortmund.de>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef AUTOWARE__SCATTER_OPS_UTILS_CUH
#define AUTOWARE__SCATTER_OPS_UTILS_CUH

#include <cuda_bf16.h>
#include <cuda_fp16.h>

template <typename scalar_t>
__global__ void fill_kernel(
  scalar_t * src, int32_t numel, const scalar_t val)  // cSpell:ignore numel
{
  int32_t thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (thread_idx >= numel) return;

  src[thread_idx] = val;
}

template <typename scalar_t>
__global__ void replace_kernel(
  scalar_t * src, int32_t numel, const scalar_t ori_val, const scalar_t new_val)
{
  int32_t thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (thread_idx >= numel) return;

  if (src[thread_idx] == ori_val) src[thread_idx] = new_val;
}

template <typename scalar_t>
__global__ void div_kernel(
  scalar_t * src, int32_t numel, const scalar_t * divisor, bool skip_zero_divisor = false)
{
  int32_t thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (thread_idx >= numel) return;

  if (skip_zero_divisor && divisor[thread_idx] == (scalar_t)0) return;

  src[thread_idx] /= divisor[thread_idx];
}

// cSpell:ignore indptr
inline __host__ __device__ int indptr_to_offset(
  const int64_t * indptr_size, int32_t indptr_dim, int32_t idx)
{
  int offset = idx % (indptr_size[indptr_dim - 1] - 1), stride = 1;
  idx /= indptr_size[indptr_dim - 1] - 1;
  for (int i = indptr_dim - 2; i >= 0; --i) {
    stride *= indptr_size[i + 1];
    offset += (idx % indptr_size[i]) * stride;
    idx /= indptr_size[i];
  }
  return offset;
}

#endif  // AUTOWARE__SCATTER_OPS_UTILS_CUH
