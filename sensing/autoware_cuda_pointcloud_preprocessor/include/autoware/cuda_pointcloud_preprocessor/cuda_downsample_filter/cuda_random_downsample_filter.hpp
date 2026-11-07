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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_HPP_

#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>
#include <memory>

namespace autoware::cuda_pointcloud_preprocessor
{

/// Uniform random downsample to a fixed point budget, on the GPU.
///
/// Mirrors `autoware::pointcloud_preprocessor::RandomDownsampleFilterComponent`:
/// at most `sample_num` points survive, chosen uniformly at random, and an input
/// already at or below the budget passes through whole. NDT cares about the
/// budget, not about which points make it, so this is the cheapest way to bound
/// scan-matching cost per frame.
///
/// The point layout is passed through untouched. Selection works on whole points
/// and copies `point_step` bytes each, so intensity, ring, timestamp and any
/// vendor field survive without this filter needing to know they exist. Unlike
/// the crop box, nothing here reads a coordinate at all — the fields are never
/// interpreted, only counted — which is why there is no layout rejection path.
class CudaRandomDownsampleFilter
{
public:
  explicit CudaRandomDownsampleFilter(std::size_t sample_num);
  ~CudaRandomDownsampleFilter();

  CudaRandomDownsampleFilter(const CudaRandomDownsampleFilter &) = delete;
  CudaRandomDownsampleFilter & operator=(const CudaRandomDownsampleFilter &) = delete;

  /// Downsample `input`, returning a new cloud with the same fields and
  /// point_step and `min(input points, sample_num)` points.
  ///
  /// Never returns nullptr: there is no layout this cannot handle. The signature
  /// still matches `CudaCropBoxFilter::filter` so the two chain interchangeably.
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filter(
    const cuda_blackboard::CudaPointCloud2 & input);

  /// The stream every kernel here runs on, so the subscriber can hand it over
  /// and avoid a synchronise between the producer and this filter.
  cudaStream_t stream() const { return stream_; }

private:
  std::size_t sample_num_;
  cudaStream_t stream_{};

  // Seeds the per-call hash. A counter rather than a clock or a device RNG
  // state: a given sequence of calls then produces a given sequence of
  // selections, which is what makes a bag replay reproducible and a failing
  // test reproducible with it. It is not a cryptographic or a statistically
  // audited stream, and does not need to be.
  std::uint32_t call_counter_{0};

  // Scratch, grown on demand and kept between calls. The sort needs a key and a
  // value array per point, and reallocating those every frame is the cost this
  // whole package exists to avoid.
  std::uint32_t * d_keys_{nullptr};
  std::uint32_t * d_indices_{nullptr};
  std::size_t capacity_{0};

  void ensureCapacity(std::size_t num_points);
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_HPP_
