// Copyright 2026 NEWSLab, National Taiwan University
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

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_random_downsample_filter.hpp"

#include <thrust/execution_policy.h>
#include <thrust/sort.h>

#include <stdexcept>
#include <string>

namespace autoware::cuda_pointcloud_preprocessor
{
namespace
{

constexpr int kBlockSize = 256;

/// A counter-based 32-bit mixer (the finaliser from Murmur3's avalanche family).
///
/// Counter-based rather than stateful, because a per-point curand state would
/// have to be allocated, initialised and carried across frames to stay
/// reproducible; a pure function of (index, seed) needs none of that and still
/// gives keys with no visible structure along the scan line.
__device__ inline std::uint32_t mix32(std::uint32_t x)
{
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}

/// Give every point a pseudo-random sort key and its own identity as the value.
__global__ void keyKernel(
  std::uint32_t * __restrict__ keys, std::uint32_t * __restrict__ indices, std::size_t num_points,
  std::uint32_t seed)
{
  const std::size_t i = blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (i >= num_points) {
    return;
  }
  // The golden-ratio stride keeps consecutive indices far apart before mixing,
  // so the low bits of i do not survive into the low bits of the key.
  keys[i] = mix32(mix32(seed) + static_cast<std::uint32_t>(i) * 0x9e3779b9U);
  indices[i] = static_cast<std::uint32_t>(i);
}

/// Copy the selected points, whole, into consecutive output slots.
__global__ void gatherKernel(
  const std::uint8_t * __restrict__ in, std::uint8_t * __restrict__ out,
  const std::uint32_t * __restrict__ indices, std::size_t kept, std::size_t point_step)
{
  const std::size_t j = blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (j >= kept) {
    return;
  }
  const std::uint8_t * src = in + static_cast<std::size_t>(indices[j]) * point_step;
  std::uint8_t * dst = out + j * point_step;
  for (std::size_t b = 0; b < point_step; ++b) {
    dst[b] = src[b];
  }
}

void check(cudaError_t err, const char * what)
{
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(err));
  }
}

}  // namespace

CudaRandomDownsampleFilter::CudaRandomDownsampleFilter(std::size_t sample_num)
: sample_num_(sample_num)
{
  check(cudaStreamCreate(&stream_), "cudaStreamCreate");
}

CudaRandomDownsampleFilter::~CudaRandomDownsampleFilter()
{
  if (d_keys_ != nullptr) {
    cudaFree(d_keys_);
  }
  if (d_indices_ != nullptr) {
    cudaFree(d_indices_);
  }
  if (stream_ != nullptr) {
    cudaStreamDestroy(stream_);
  }
}

void CudaRandomDownsampleFilter::ensureCapacity(std::size_t num_points)
{
  if (num_points <= capacity_) {
    return;
  }
  if (d_keys_ != nullptr) {
    cudaFree(d_keys_);
    d_keys_ = nullptr;
  }
  if (d_indices_ != nullptr) {
    cudaFree(d_indices_);
    d_indices_ = nullptr;
  }
  check(cudaMalloc(&d_keys_, num_points * sizeof(std::uint32_t)), "cudaMalloc keys");
  check(cudaMalloc(&d_indices_, num_points * sizeof(std::uint32_t)), "cudaMalloc indices");
  capacity_ = num_points;
}

// Algorithm: random key, full sort, take the prefix.
//
// Give point i a pseudo-random key, sort the index array by that key, and keep
// the first `sample_num` indices. That yields an EXACT count, which is the
// property worth paying for: the CPU component promises "at most sample_num",
// and everything downstream — NDT's per-frame budget, the memory the
// concatenator sizes — is written against that promise.
//
// The obvious cheaper alternative is thresholding: keep point i when its key is
// below sample_num/N * UINT32_MAX. That is a single kernel plus the scan the
// crop box already uses, no sort at all, but the kept count is binomial around
// sample_num rather than equal to it — a frame can come out 3% over budget, and
// a nearly-empty frame can come out empty. Since this runs once per scan on a
// few hundred thousand points, the sort is not the bottleneck (the driver and
// the NDT iterations are), so exactness is the better trade here.
//
// Two details that are deliberate rather than incidental:
//
// - The selected indices are sorted ascending again before the gather, so the
//   output preserves input order. Nothing in NDT requires it, but a cloud whose
//   points still run in scan order stays readable in RViz and diffable against
//   the input, and the second sort is over `sample_num` elements, not N.
//
// - 32-bit keys collide by the birthday bound well before the ~10^5 points a
//   VLP-32C scan carries, so some keys tie. Ties are broken arbitrarily by the
//   sort, which perturbs uniformity slightly among tied points; it cannot
//   perturb the count, which stays exactly sample_num.
std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaRandomDownsampleFilter::filter(
  const cuda_blackboard::CudaPointCloud2 & input)
{
  const std::size_t point_step = input.point_step;
  const std::size_t num_points =
    static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height);

  auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  output->header = input.header;
  output->fields = input.fields;
  output->point_step = input.point_step;
  output->is_bigendian = input.is_bigendian;
  // Unlike the crop box, this filter never inspects a coordinate, so it neither
  // removes nor introduces invalid points: whatever the input claimed about
  // density is still true of a random subset of it.
  output->is_dense = input.is_dense;
  output->height = 1;

  if (num_points == 0 || point_step == 0) {
    output->width = 0;
    output->row_step = 0;
    output->data = cuda_blackboard::CudaUniquePtr<std::uint8_t[]>();
    return output;
  }

  const std::size_t kept = num_points < sample_num_ ? num_points : sample_num_;

  output->width = static_cast<std::uint32_t>(kept);
  output->row_step = static_cast<std::uint32_t>(kept * point_step);

  // Allocated through the blackboard's own helper so the buffer carries the
  // deleter the library expects; a raw cudaMalloc here would be freed by a
  // deleter that did not allocate it.
  output->data = cuda_blackboard::make_unique<std::uint8_t[]>(kept * point_step);
  std::uint8_t * out_raw = output->data.get();

  if (kept == num_points) {
    // Everything survives, so there is nothing to choose. Skipping the sort here
    // is not just an optimisation: it also makes the pass-through case bit-exact
    // and order-preserving, which is what the CPU component does.
    check(
      cudaMemcpyAsync(
        out_raw, input.data.get(), kept * point_step, cudaMemcpyDeviceToDevice, stream_),
      "copy pass-through");
    check(cudaStreamSynchronize(stream_), "sync after pass-through");
    return output;
  }

  ensureCapacity(num_points);

  const int blocks = static_cast<int>((num_points + kBlockSize - 1) / kBlockSize);
  keyKernel<<<blocks, kBlockSize, 0, stream_>>>(d_keys_, d_indices_, num_points, call_counter_++);
  check(cudaGetLastError(), "keyKernel");

  thrust::sort_by_key(thrust::cuda::par.on(stream_), d_keys_, d_keys_ + num_points, d_indices_);
  thrust::sort(thrust::cuda::par.on(stream_), d_indices_, d_indices_ + kept);

  const int gather_blocks = static_cast<int>((kept + kBlockSize - 1) / kBlockSize);
  gatherKernel<<<gather_blocks, kBlockSize, 0, stream_>>>(
    input.data.get(), out_raw, d_indices_, kept, point_step);
  check(cudaGetLastError(), "gatherKernel");
  check(cudaStreamSynchronize(stream_), "sync after gather");

  return output;
}

}  // namespace autoware::cuda_pointcloud_preprocessor
