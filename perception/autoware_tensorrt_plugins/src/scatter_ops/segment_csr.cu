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

#include <numeric>
#include <string>
#include <tuple>
#include <vector>

#define THREADS 256
#define BLOCKS(TB, N) (TB * N + THREADS - 1) / THREADS
#define FULL_MASK 0xffffffff
#define SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, R)                                             \
  template int32_t segment_csr_launch<T, R>(                                                  \
    const T * src, const std::vector<int32_t> & src_size, const int64_t * indptr,             \
    const std::vector<int32_t> & indptr_size, std::tuple<T *, int64_t *> out,                 \
    cudaStream_t stream);                                                                     \
  template int32_t segment_csr_launch<T, R>(                                                  \
    const T * src, const std::vector<int32_t> & src_size, const int64_t * indptr,             \
    const std::vector<int32_t> & indptr_size, const T * base, std::tuple<T *, int64_t *> out, \
    cudaStream_t stream);
#define SEGMENT_CSR_LAUNCH_INSTANTIATION(T)                   \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::SUM)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MEAN) \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MUL)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::DIV)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MIN)  \
  SEGMENT_CSR_LAUNCH_INSTANTIATION_TR(T, ReductionType::MAX)

template <typename scalar_t, ReductionType REDUCE, int TB>
__global__ void segment_csr_kernel(
  const scalar_t * src, const int64_t * indptr, const int64_t * indptr_size, int32_t indptr_dim,
  scalar_t * out, int64_t * arg_out, size_t N, size_t E)
{
  // Each warp processes exactly `32/TB` rows and aggregates all row values
  // via a parallel reduction.

  int thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int row_idx = thread_idx / TB;
  int lane_idx = thread_idx & (TB - 1);
  if (row_idx >= N) return;

  int offset = indptr_to_offset(indptr_size, indptr_dim, row_idx);
  int64_t row_start = __ldg(indptr + offset);
  int64_t row_end = __ldg(indptr + offset + 1);

  scalar_t val = Reducer<scalar_t, REDUCE>::init();
  int64_t arg, arg_tmp;

  offset = (row_idx / (indptr_size[indptr_dim - 1] - 1)) * E;
  for (int64_t src_idx = row_start + lane_idx; src_idx < row_end; src_idx += TB)
    Reducer<scalar_t, REDUCE>::update(&val, src[offset + src_idx], &arg, src_idx);

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
  const scalar_t * src, const int64_t * indptr, const int64_t * indptr_size, int32_t indptr_dim,
  scalar_t * out, int64_t * arg_out, size_t N, size_t K, size_t E)
{
  // Each thread processes exactly one row. It turned out that is more
  // efficient than using shared memory due to avoiding synchronization
  // barriers.

  int thread_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int row_idx = thread_idx / K;
  int lane_idx = thread_idx % K;
  if (thread_idx >= N * K) return;

  int offset = indptr_to_offset(indptr_size, indptr_dim, row_idx);
  int64_t row_start = __ldg(indptr + offset);
  int64_t row_end = __ldg(indptr + offset + 1);

  scalar_t val = Reducer<scalar_t, REDUCE>::init();
  int64_t arg;

  offset = (row_idx / (indptr_size[indptr_dim - 1] - 1)) * E * K;
  for (int64_t src_idx = row_start; src_idx < row_end; src_idx++)
    Reducer<scalar_t, REDUCE>::update(&val, src[offset + K * src_idx + lane_idx], &arg, src_idx);

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
  const scalar_t * src, const std::vector<int32_t> & src_size, const int64_t * indptr,
  const std::vector<int32_t> & indptr_size, const scalar_t * base,
  std::tuple<scalar_t *, int64_t *> out, cudaStream_t stream)
{
  if (src_size.size() < indptr_size.size()) return -1;

  if (!std::equal(indptr_size.begin(), indptr_size.end() - 1, src_size.begin())) return -1;

  auto dim = indptr_size.size() - 1;

  auto _mul = [](int a, int b) { return a * b; };
  auto src_numel = std::accumulate(src_size.begin(), src_size.end(), 1, _mul);
  auto indptr_numel = std::accumulate(indptr_size.begin(), indptr_size.end(), 1, _mul);
  auto out_numel = src_numel / src_size[dim] * std::max<int32_t>(indptr_size[dim] - 1, 0);

  cudaMemcpyAsync(
    std::get<0>(out), base, sizeof(scalar_t) * out_numel, cudaMemcpyDeviceToDevice, stream);

  if ((REDUCE == ReductionType::MIN || REDUCE == ReductionType::MAX) && std::get<1>(out) != nullptr)
    fill_kernel<int64_t>
      <<<BLOCKS(1, out_numel), THREADS, 0, stream>>>(std::get<1>(out), out_numel, src_size[dim]);

  if (src_numel == 0) return 0;

  auto N = max(indptr_size[dim] - 1, 0) * (indptr_numel / indptr_size[dim]);
  auto K = out_numel / N;
  auto E = src_size[dim];
  int64_t * indptr_size_dev;
  cudaMallocAsync(&indptr_size_dev, sizeof(int64_t) * indptr_size.size(), stream);
  cudaMemcpyAsync(
    indptr_size_dev, indptr_size.data(), sizeof(int64_t) * indptr_size.size(),
    cudaMemcpyHostToDevice, stream);

  if (K == 1)
    segment_csr_kernel<scalar_t, REDUCE, 1><<<BLOCKS(32, N), THREADS, 0, stream>>>(
      src, indptr, indptr_size_dev, indptr_size.size(), std::get<0>(out), std::get<1>(out), N, E);
  else
    segment_csr_broadcast_kernel<scalar_t, REDUCE><<<BLOCKS(1, N * K), THREADS, 0, stream>>>(
      src, indptr, indptr_size_dev, indptr_size.size(), std::get<0>(out), std::get<1>(out), N, K,
      E);

  cudaFreeAsync(indptr_size_dev, stream);
  return 0;
}

template <typename scalar_t, ReductionType REDUCE>
int32_t segment_csr_launch(
  const scalar_t * src, const std::vector<int32_t> & src_size, const int64_t * indptr,
  const std::vector<int32_t> & indptr_size, std::tuple<scalar_t *, int64_t *> out,
  cudaStream_t stream)
{
  auto dim = indptr_size.size() - 1;
  auto src_numel =
    std::accumulate(src_size.begin(), src_size.end(), 1, [](int a, int b) { return a * b; });
  auto out_numel = src_numel / src_size[dim] * std::max<int32_t>(indptr_size[dim] - 1, 0);

  scalar_t * base;
  cudaMallocAsync(&base, sizeof(scalar_t) * out_numel, stream);
  fill_kernel<scalar_t><<<BLOCKS(1, out_numel), THREADS, 0, stream>>>(base, out_numel, (scalar_t)0);

  auto status =
    segment_csr_launch<scalar_t, REDUCE>(src, src_size, indptr, indptr_size, base, out, stream);
  if (status != 0) return status;

  cudaFreeAsync(base, stream);
  return 0;
}

SEGMENT_CSR_LAUNCH_INSTANTIATION(half)
SEGMENT_CSR_LAUNCH_INSTANTIATION(float)
