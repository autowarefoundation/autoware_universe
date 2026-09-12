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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CROP_BOX_FILTER__CUDA_CROP_BOX_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CROP_BOX_FILTER__CUDA_CROP_BOX_FILTER_HPP_

#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cuda_runtime.h>

#include <memory>

namespace autoware::cuda_pointcloud_preprocessor
{

/// Axis-aligned crop box, on the GPU.
///
/// Mirrors `autoware::pointcloud_preprocessor::CropBoxFilterComponent`: keep the
/// points inside the box, or with `negative` keep the ones outside it. The
/// bounds are inclusive on both ends, matching the CPU filter — a point exactly
/// on a face is inside.
///
/// The point layout is passed through untouched. Cropping selects whole points
/// and copies `point_step` bytes each, so intensity, ring, timestamp and any
/// vendor field survive without this filter needing to know they exist. Only the
/// x, y and z offsets are read, which is why the input's fields are inspected
/// rather than assumed.
///
/// No frame transform is performed. The CPU component can crop in a frame other
/// than the cloud's own and looks up tf to do it; here the box is required to be
/// in the cloud's frame, and the node rejects a mismatch rather than silently
/// cropping the wrong region. The localization chain uses base_link for both.
class CudaCropBoxFilter
{
public:
  struct BoxParams
  {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    bool negative;
  };

  explicit CudaCropBoxFilter(const BoxParams & params);
  ~CudaCropBoxFilter();

  CudaCropBoxFilter(const CudaCropBoxFilter &) = delete;
  CudaCropBoxFilter & operator=(const CudaCropBoxFilter &) = delete;

  /// Crop `input`, returning a new cloud with the same fields and point_step.
  ///
  /// Returns nullptr when the input carries no usable x/y/z float32 fields; the
  /// caller is expected to log and drop the message rather than publish a cloud
  /// built from a layout this cannot read.
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filter(
    const cuda_blackboard::CudaPointCloud2 & input);

  /// The stream every kernel here runs on, so the subscriber can hand it over
  /// and avoid a synchronise between the producer and this filter.
  cudaStream_t stream() const { return stream_; }

  /// Offsets of the x, y, z fields, or false when the layout is unusable.
  static bool findXyzOffsets(
    const cuda_blackboard::CudaPointCloud2 & cloud, std::size_t offsets[3]);

private:
  BoxParams params_;
  cudaStream_t stream_{};

  // Scratch, grown on demand and kept between calls. A scan needs somewhere to
  // put the per-point mask and its prefix sum, and reallocating those every
  // scan is the cost this whole package exists to avoid.
  std::uint32_t * d_mask_{nullptr};
  std::uint32_t * d_indices_{nullptr};
  std::size_t capacity_{0};

  void ensureCapacity(std::size_t num_points);
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CROP_BOX_FILTER__CUDA_CROP_BOX_FILTER_HPP_
