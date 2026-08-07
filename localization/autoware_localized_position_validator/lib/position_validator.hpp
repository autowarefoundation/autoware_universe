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

#ifndef AUTOWARE__LOCALIZED_POSITION_VALIDATOR__POSITION_VALIDATOR_HPP_
#define AUTOWARE__LOCALIZED_POSITION_VALIDATOR__POSITION_VALIDATOR_HPP_

#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <ATen/ATen.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cuda_runtime_api.h>

#include <memory>
#include <vector>

namespace autoware::localized_position_validator
{
using autoware::tensorrt_common::TrtCommonConfig;

class PositionValidatorTRT
{
public:
  explicit PositionValidatorTRT(
    const TrtCommonConfig & model_param,
    const std::vector<float> & voxel_size, const std::vector<float> & coors_range,
    const int & max_voxels, const int & max_points_per_voxel, const int & profile_points_min,
    const int & profile_points_opt, const int & profile_points_max);

  virtual ~PositionValidatorTRT();

  bool inference(const std::vector<Eigen::Vector4f> & points,
    int & predicted_class, float & confidence);

private:
  void preprocess(
    const std::vector<Eigen::Vector4f> & points
  );

  std::vector<float> voxel_size_{};
  std::vector<float> coors_range_{};
  int max_voxels_;
  int max_points_per_voxel_;
  int profile_points_min_;
  int profile_points_max_;

  at::Tensor pillars_;
  at::Tensor coors_batch_;
  at::Tensor npoints_per_pillar_;
  at::Tensor output_tensor_ = at::empty({1, 2},
    at::TensorOptions().dtype(at::kFloat).device(at::kCUDA).requires_grad(false)
  );
  std::unique_ptr<tensorrt_common::TrtCommon> model_trt_ptr_{nullptr};
  cudaStream_t stream_{nullptr};
};

}  // namespace autoware::localized_position_validator

#endif  // AUTOWARE__LOCALIZED_POSITION_VALIDATOR__POSITION_VALIDATOR_HPP_
