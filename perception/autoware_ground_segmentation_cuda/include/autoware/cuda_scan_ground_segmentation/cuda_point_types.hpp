// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_POINT_TYPES_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_POINT_TYPES_HPP_

namespace autoware::cuda
{

// TODO: this is not a
struct alignas(16) PointXYZ
{
  float x, y, z;
};

struct alignas(16) PointXYZI
{
  float x, y, z, intensity;
};

}  // namespace autoware::cuda

#endif  // AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_POINT_TYPES_HPP_
