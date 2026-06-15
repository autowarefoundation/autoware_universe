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

#include <algorithm>
#include <string>

#define THREADS 256
#define BLOCKS(TB, N) (TB * N + THREADS - 1) / THREADS
#define FULL_MASK 0xffffffff
#define SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, R)                                          \
  template int32_t segment_csr_launch<T, R>(                                               \
    const T * src_in, int32_t num_rows_in, int32_t num_cols_in, const int64_t * indptr_in, \
    int32_t indptr_size_in, T * reduced_values_out, int64_t * arg_indices_out,             \
    cudaStream_t stream_in);
#define SEGMENT_CSR_LAUNCH_INSTANTIATION(T)                   \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::SUM)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MEAN) \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MUL)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::DIV)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MIN)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MAX)

template <typename scalar_t, ReductionType REDUCE, int TB>
__global__ void segment_csr_kernel(
  const scalar_t * src_in, const int64_t * indptr_in, scalar_t * reduced_values_out,
  int64_t * arg_indices_out, size_t num_segments_in)
{
  // Each warp processes exactly `32/TB` rows and aggregates all row values
  // via a parallel reduction.

  int thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int row_idx = thread_idx / TB;
  int lane_idx = thread_idx & (TB - 1);
  if (row_idx >= num_segments_in) return;

  int64_t row_start = __ldg(indptr_in + row_idx);
  int64_t row_end = __ldg(indptr_in + row_idx + 1);

  scalar_t val = Reducer<scalar_t, REDUCE>::init();
  int64_t arg{0}, arg_tmp{0};

  for (int64_t src_idx = row_start + lane_idx; src_idx < row_end; src_idx += TB)
    Reducer<scalar_t, REDUCE>::update(&val, src_in[src_idx], &arg, src_idx);

#pragma unroll
  for (int i = TB / 2; i > 0; i /= 2) {
    // Parallel reduction inside a single warp.
    if (REDUCE == ReductionType::MIN || REDUCE == ReductionType::MAX)
      arg_tmp = __shfl_down_sync(FULL_MASK, arg, i);
    Reducer<scalar_t, REDUCE>::update(&val, __shfl_down_sync(FULL_MASK, val, i), &arg, arg_tmp);
  }

  if (lane_idx == 0)
    if (arg_indices_out != nullptr)
      Reducer<scalar_t, REDUCE>::write(
        reduced_values_out + row_idx, val, arg_indices_out + row_idx, arg, row_end - row_start);
    else
      Reducer<scalar_t, REDUCE>::write(reduced_values_out + row_idx, val, row_end - row_start);
}

template <typename scalar_t, ReductionType REDUCE>
__global__ void segment_csr_broadcast_kernel(
  const scalar_t * src_in, const int64_t * indptr_in, scalar_t * reduced_values_out,
  int64_t * arg_indices_out, size_t num_segments_in, size_t num_cols_in)
{
  // Each thread processes exactly one row. It turned out that is more
  // efficient than using shared memory due to avoiding synchronization
  // barriers.

  int thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int row_idx = thread_idx / num_cols_in;
  int lane_idx = thread_idx % num_cols_in;
  if (thread_idx >= num_segments_in * num_cols_in) return;

  int64_t row_start = __ldg(indptr_in + row_idx);
  int64_t row_end = __ldg(indptr_in + row_idx + 1);

  scalar_t val = Reducer<scalar_t, REDUCE>::init();
  int64_t arg{0};

  for (int64_t src_idx = row_start; src_idx < row_end; src_idx++)
    Reducer<scalar_t, REDUCE>::update(
      &val, src_in[num_cols_in * src_idx + lane_idx], &arg, src_idx);

  if (arg_indices_out != nullptr)
    Reducer<scalar_t, REDUCE>::write(
      reduced_values_out + thread_idx, val, arg_indices_out + thread_idx, arg, row_end - row_start);
  else
    Reducer<scalar_t, REDUCE>::write(reduced_values_out + thread_idx, val, row_end - row_start);
}

//! \todo test different devices (cudaSetDevice(src.get_device());)
//! \todo expand index
template <typename scalar_t, ReductionType REDUCE>
int32_t segment_csr_launch(
  const scalar_t * src_in, int32_t num_rows_in, int32_t num_cols_in, const int64_t * indptr_in,
  int32_t indptr_size_in, scalar_t * reduced_values_out, int64_t * arg_indices_out,
  cudaStream_t stream_in)
{
  if (num_rows_in < 0 || num_cols_in < 0 || indptr_size_in < 0) return -1;

  auto num_segments = std::max<int32_t>(indptr_size_in - 1, 0);
  auto out_numel = static_cast<size_t>(num_segments) * static_cast<size_t>(num_cols_in);

  if (out_numel == 0) return 0;

  if ((REDUCE == ReductionType::MIN || REDUCE == ReductionType::MAX) && arg_indices_out != nullptr)
    fill_kernel<int64_t>
      <<<BLOCKS(1, out_numel), THREADS, 0, stream_in>>>(arg_indices_out, out_numel, num_rows_in);

  scalar_t * base_values{nullptr};
  cudaMallocAsync(&base_values, sizeof(scalar_t) * out_numel, stream_in);
  fill_kernel<scalar_t><<<BLOCKS(1, out_numel), THREADS, 0, stream_in>>>(
    base_values, out_numel, static_cast<scalar_t>(0));
  cudaMemcpyAsync(
    reduced_values_out, base_values, sizeof(scalar_t) * out_numel, cudaMemcpyDeviceToDevice,
    stream_in);

  if (num_cols_in == 1)
    segment_csr_kernel<scalar_t, REDUCE, 1><<<BLOCKS(32, num_segments), THREADS, 0, stream_in>>>(
      src_in, indptr_in, reduced_values_out, arg_indices_out, num_segments);
  else
    segment_csr_broadcast_kernel<scalar_t, REDUCE>
      <<<BLOCKS(1, num_segments * num_cols_in), THREADS, 0, stream_in>>>(
        src_in, indptr_in, reduced_values_out, arg_indices_out, num_segments, num_cols_in);

  cudaFreeAsync(base_values, stream_in);
  return 0;
}

SEGMENT_CSR_LAUNCH_INSTANTIATION(half)
SEGMENT_CSR_LAUNCH_INSTANTIATION(float)
