// MIT License
//
// Copyright (c) 2023 ZhuLifa
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__LOCALIZED_POSITION_VALIDATOR__PREPROCESS__VOXELIZATION_HPP_
#define AUTOWARE__LOCALIZED_POSITION_VALIDATOR__PREPROCESS__VOXELIZATION_HPP_

#include <ATen/ATen.h>

typedef enum { SUM = 0, MEAN = 1, MAX = 2 } reduce_t;

namespace autoware::localized_position_validator {
namespace voxelization {

int hard_voxelize(
  const at::Tensor & points, at::Tensor & voxels, at::Tensor & coors,
  at::Tensor & num_points_per_voxel, const std::vector<float> & voxel_size,
  const std::vector<float> & coors_range, const int & max_points, const int & max_voxels,
  const int & NDim, const bool & deterministic);

int hard_voxelize_cpu(
  const at::Tensor & points, at::Tensor & voxels, at::Tensor & coors,
  at::Tensor & num_points_per_voxel, const std::vector<float> & voxel_size,
  const std::vector<float> & coors_range, const int & max_points, const int & max_voxels,
  const int & NDim = 3);

int hard_voxelize_gpu(
  const at::Tensor & points, at::Tensor & voxels, at::Tensor & coors,
  at::Tensor & num_points_per_voxel, const std::vector<float> & voxel_size,
  const std::vector<float> & coors_range, const int & max_points, const int & max_voxels,
  const int & NDim = 3);

int nondisterministic_hard_voxelize_gpu(
  const at::Tensor & points, at::Tensor & voxels, at::Tensor & coors,
  at::Tensor & num_points_per_voxel, const std::vector<float> & voxel_size,
  const std::vector<float> & coors_range, const int & max_points, const int & max_voxels,
  const int & NDim = 3);

inline reduce_t convert_reduce_type(const std::string & reduce_type) {
  if (reduce_type == "max")
    return reduce_t::MAX;
  else if (reduce_type == "sum")
    return reduce_t::SUM;
  else if (reduce_type == "mean")
    return reduce_t::MEAN;
  else TORCH_CHECK(false, "do not support reduce type " + reduce_type)
  return reduce_t::SUM;
}

}  // namespace voxelization
}  // namespace autoware::localized_position_validator

#endif  // AUTOWARE__LOCALIZED_POSITION_VALIDATOR__PREPROCESS__VOXELIZATION_HPP_
