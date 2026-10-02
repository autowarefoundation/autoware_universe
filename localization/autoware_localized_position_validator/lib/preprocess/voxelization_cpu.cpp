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

#include <ATen/TensorUtils.h>
#include <torch/torch.h>

namespace {

template <typename T, typename T_int>
void dynamic_voxelize_kernel(const torch::TensorAccessor<T, 2> points,
  torch::TensorAccessor<T_int, 2> coors, const std::vector<float> & voxel_size,
  const std::vector<float> & coors_range, const std::vector<int> & grid_size,
  const int & num_points, const int & NDim)
{
  const int ndim_minus_1 = NDim - 1;
  int* coor = new int[NDim]();
  int c;

  for (int i = 0; i < num_points; ++i) {
    bool failed = false;
    for (int j = 0; j < NDim; ++j) {
      c = floor((points[i][j] - coors_range[j]) / voxel_size[j]);
      // necessary to rm points out of range
      if ((c < 0 || c >= grid_size[j])) {
        failed = true;
        break;
      }
      coor[ndim_minus_1 - j] = c;
    }

    for (int k = 0; k < NDim; ++k) {
      if (failed)
        coors[i][k] = -1;
      else
        coors[i][k] = coor[k];
    }
  }

  delete[] coor;
  return;
}

template <typename T, typename T_int>
void hard_voxelize_kernel(
  const torch::TensorAccessor<T, 2> points, torch::TensorAccessor<T, 3> voxels,
  torch::TensorAccessor<T_int, 2> coors, torch::TensorAccessor<T_int, 1> num_points_per_voxel,
  torch::TensorAccessor<T_int, 3> coor_to_voxelidx, int & voxel_num,
  const std::vector<float> & voxel_size, const std::vector<float> & coors_range,
  const std::vector<int> & grid_size, const int & max_points, const int & max_voxels,
  const int & num_points, const int & num_features, const int & NDim)
{
  // declare a temp coors
  at::Tensor temp_coors = at::zeros(
    {num_points, NDim}, at::TensorOptions().dtype(at::kInt).device(at::kCPU));

  // First use dynamic voxelization to get coors,
  // then check max points/voxels constraints
  dynamic_voxelize_kernel<T, int>(points, temp_coors.accessor<int, 2>(), voxel_size, coors_range,
    grid_size, num_points, NDim);

  int voxelidx, num;
  auto coor = temp_coors.accessor<int, 2>();

  for (int i = 0; i < num_points; ++i) {
    if (coor[i][0] == -1) continue;

    voxelidx = coor_to_voxelidx[coor[i][0]][coor[i][1]][coor[i][2]];

    // record voxel
    if (voxelidx == -1) {
      voxelidx = voxel_num;
      if (max_voxels != -1 && voxel_num >= max_voxels) continue;
      voxel_num += 1;

      coor_to_voxelidx[coor[i][0]][coor[i][1]][coor[i][2]] = voxelidx;

      for (int k = 0; k < NDim; ++k) {
        coors[voxelidx][k] = coor[i][k];
      }
    }

    // put points into voxel
    num = num_points_per_voxel[voxelidx];
    if (max_points == -1 || num < max_points) {
      for (int k = 0; k < num_features; ++k) {
        voxels[voxelidx][num][k] = points[i][k];
      }
      num_points_per_voxel[voxelidx] += 1;
    }
  }

  return;
}

}  // namespace

namespace autoware::localized_position_validator {
namespace voxelization {

int hard_voxelize_cpu(
  const at::Tensor & points, at::Tensor & voxels, at::Tensor & coors,
  at::Tensor & num_points_per_voxel, const std::vector<float> & voxel_size,
  const std::vector<float> & coors_range, const int & max_points, const int & max_voxels,
  const int & NDim = 3)
{
  // current version tooks about 0.02s_0.03s for one frame on cpu
  // check device
  AT_ASSERTM(points.device().is_cpu(), "points must be a CPU tensor");

  std::vector<int> grid_size(NDim);
  const int num_points = points.size(0);
  const int num_features = points.size(1);

  for (int i = 0; i < NDim; ++i) {
    grid_size[i] =
      round((coors_range[NDim + i] - coors_range[i]) / voxel_size[i]);
  }

  // coors, num_points_per_voxel, coor_to_voxelidx are int Tensor
  at::Tensor coor_to_voxelidx =
    -at::ones({grid_size[2], grid_size[1], grid_size[0]}, coors.options());

  int voxel_num = 0;
  AT_DISPATCH_FLOATING_TYPES_AND_HALF(
    points.scalar_type(), "hard_voxelize_forward", [&] {
      hard_voxelize_kernel<scalar_t, int>(
        points.accessor<scalar_t, 2>(), voxels.accessor<scalar_t, 3>(),
        coors.accessor<int, 2>(), num_points_per_voxel.accessor<int, 1>(),
        coor_to_voxelidx.accessor<int, 3>(), voxel_num, voxel_size,
        coors_range, grid_size, max_points, max_voxels, num_points,
        num_features, NDim);
    });

  return voxel_num;
}

void dynamic_voxelize_cpu(
  const at::Tensor & points, at::Tensor & coors, const std::vector<float> & voxel_size,
  const std::vector<float> & coors_range, const int & NDim = 3)
{
  // check device
  AT_ASSERTM(points.device().is_cpu(), "points must be a CPU tensor");

  std::vector<int> grid_size(NDim);
  const int num_points = points.size(0);

  for (int i = 0; i < NDim; ++i) {
    grid_size[i] =
      round((coors_range[NDim + i] - coors_range[i]) / voxel_size[i]);
  }

  // coors, num_points_per_voxel, coor_to_voxelidx are int Tensor
  AT_DISPATCH_FLOATING_TYPES_AND_HALF(
    points.scalar_type(), "hard_voxelize_forward", [&] {
      dynamic_voxelize_kernel<scalar_t, int>(
        points.accessor<scalar_t, 2>(), coors.accessor<int, 2>(),
        voxel_size, coors_range, grid_size, num_points, NDim);
    });

  return;
}

}  // namespace voxelization
}  // autoware::localized_position_validator
