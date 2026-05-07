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

#include "autoware/scatter_ops/reduction.cuh"
#include "autoware/scatter_ops/reduction.h"
#include "autoware/scatter_ops/segment_csr.h"
#include "autoware/scatter_ops/utils.cuh"

#include <string>
#include <tuple>

#define THREADS 256
#define BLOCKS(TB, N) (TB * N + THREADS - 1) / THREADS
#define FULL_MASK 0xffffffff
#define SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, R)                             \
  template int32_t segment_csr_launch<T, R>(                                  \
    const T * src, int32_t num_rows, int32_t num_cols, const int64_t * indptr, \
    int32_t indptr_size, std::tuple<T *, int64_t *> out, cudaStream_t stream);
#define SEGMENT_CSR_LAUNCH_INSTANTIATION(T)                   \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::SUM)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MEAN) \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MUL)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::DIV)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MIN)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MAX)

template <typename scalar_t, ReductionType REDUCE, int TB>
__global__ void segment_csr_kernel(
  const scalar_t * src, const int64_t * indptr, scalar_t * out, int64_t * arg_out,
  size_t num_segments)
{
  // Each warp processes exactly `32/TB` rows and aggregates all row values
  // via a parallel reduction.

  int thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int row_idx = thread_idx / TB;
  int lane_idx = thread_idx & (TB - 1);
  if (row_idx >= num_segments) return;

  int64_t row_start = __ldg(indptr + row_idx);
  int64_t row_end = __ldg(indptr + row_idx + 1);

  scalar_t val = Reducer<scalar_t, REDUCE>::init();
  int64_t arg{0}, arg_tmp{0};

  for (int64_t src_idx = row_start + lane_idx; src_idx < row_end; src_idx += TB)
    Reducer<scalar_t, REDUCE>::update(&val, src[src_idx], &arg, src_idx);

#pragma unroll
  for (int i = TB / 2; i > 0; i /= 2) {
    // Parallel reduction inside a single warp.
    if (REDUCE == ReductionType::MIN || REDUCE == ReductionType::MAX)
      arg_tmp = __shfl_down_sync(FULL_MASK, arg, i);
    Reducer<scalar_t, REDUCE>::update(&val, __shfl_down_sync(FULL_MASK, val, i), &arg, arg_tmp);
  }

  if (lane_idx == 0)
    if (arg_out != nullptr)
      Reducer<scalar_t, REDUCE>::write(
        out + row_idx, val, arg_out + row_idx, arg, row_end - row_start);
    else
      Reducer<scalar_t, REDUCE>::write(out + row_idx, val, row_end - row_start);
}

template <typename scalar_t, ReductionType REDUCE>
__global__ void segment_csr_broadcast_kernel(
  const scalar_t * src, const int64_t * indptr, scalar_t * out, int64_t * arg_out,
  size_t num_segments, size_t num_cols)
{
  // Each thread processes exactly one row. It turned out that is more
  // efficient than using shared memory due to avoiding synchronization
  // barriers.

  int thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int row_idx = thread_idx / num_cols;
  int lane_idx = thread_idx % num_cols;
  if (thread_idx >= num_segments * num_cols) return;

  int64_t row_start = __ldg(indptr + row_idx);
  int64_t row_end = __ldg(indptr + row_idx + 1);

  scalar_t val = Reducer<scalar_t, REDUCE>::init();
  int64_t arg{0};

  for (int64_t src_idx = row_start; src_idx < row_end; src_idx++)
    Reducer<scalar_t, REDUCE>::update(&val, src[num_cols * src_idx + lane_idx], &arg, src_idx);

  if (arg_out != nullptr)
    Reducer<scalar_t, REDUCE>::write(
      out + thread_idx, val, arg_out + thread_idx, arg, row_end - row_start);
  else
    Reducer<scalar_t, REDUCE>::write(out + thread_idx, val, row_end - row_start);
}

//! \todo test different devices (cudaSetDevice(src.get_device());)
//! \todo expand index
template <typename scalar_t, ReductionType REDUCE>
int32_t segment_csr_launch(
  const scalar_t * src, int32_t num_rows, int32_t num_cols, const int64_t * indptr,
  int32_t indptr_size, std::tuple<scalar_t *, int64_t *> out, cudaStream_t stream)
{
  if (num_rows < 0 || num_cols < 0 || indptr_size < 0) return -1;

  auto num_segments = std::max<int32_t>(indptr_size - 1, 0);
  auto out_numel = static_cast<size_t>(num_segments) * static_cast<size_t>(num_cols);
  if ((REDUCE == ReductionType::MIN || REDUCE == ReductionType::MAX) && std::get<1>(out) != nullptr)
    fill_kernel<int64_t>
      <<<BLOCKS(1, out_numel), THREADS, 0, stream>>>(std::get<1>(out), out_numel, num_rows);

  if (num_segments == 0 || num_cols == 0) return 0;

  fill_kernel<scalar_t><<<BLOCKS(1, out_numel), THREADS, 0, stream>>>(
    std::get<0>(out), out_numel, static_cast<scalar_t>(0));

  if (num_cols == 1)
    segment_csr_kernel<scalar_t, REDUCE, 1><<<BLOCKS(32, num_segments), THREADS, 0, stream>>>(
      src, indptr, std::get<0>(out), std::get<1>(out), num_segments);
  else
    segment_csr_broadcast_kernel<scalar_t, REDUCE>
      <<<BLOCKS(1, num_segments * num_cols), THREADS, 0, stream>>>(
        src, indptr, std::get<0>(out), std::get<1>(out), num_segments, num_cols);
  return 0;
}

SEGMENT_CSR_LAUNCH_INSTANTIATION(half)
SEGMENT_CSR_LAUNCH_INSTANTIATION(float)
