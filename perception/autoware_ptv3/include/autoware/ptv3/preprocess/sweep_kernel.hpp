// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__PTV3__PREPROCESS__SWEEP_KERNEL_HPP_
#define AUTOWARE__PTV3__PREPROCESS__SWEEP_KERNEL_HPP_

#include "autoware/ptv3/preprocess/point_type.hpp"

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>

namespace autoware::ptv3
{

/// Column-major 4x4 past-lidar-frame to current-lidar-frame transform, passed to the kernel by
/// value so aggregating a frame needs no device copy and no stream synchronization.
struct SweepTransform
{
  float matrix[16];
};

/// Ego-motion-compensate one cached lidar frame and write its network features.
///
/// Every input point produces one `(x, y, z, intensity, time_lag)` row of
/// `num_features` floats in `output_points`. The xyz coordinates are transformed
/// by `transform` (past lidar frame -> current lidar frame; identity for the
/// current frame). Intensity keeps the format-specific
/// normalization used in training (`/255` for the 8-bit intensity formats, raw
/// for the float intensity formats). `time_lag` is `0` for the current frame and
/// the age in seconds for sweeps. `is_current_frame` marks the frame the sweeps
/// are compensated into; only sweeps drop their ego ghosts.
void generateSweepFeaturesLaunch(
  const void * input_data, CloudFormat input_format, std::size_t num_points, float time_lag,
  bool is_current_frame, float close_radius, SweepTransform transform, std::int64_t num_features,
  float * output_points, std::uint32_t threads_per_block, cudaStream_t stream);

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PREPROCESS__SWEEP_KERNEL_HPP_
