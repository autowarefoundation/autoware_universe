// Copyright 2025 TIER IV.
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

#ifndef CORE__SENSOR_TYPES_HPP_
#define CORE__SENSOR_TYPES_HPP_

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad
{

struct CoreCameraInfo
{
  int32_t width{0};
  int32_t height{0};
  std::array<double, 9> k{};
  std::string frame_id;
};

struct CoreKinematicState
{
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d linear_velocity{0.0, 0.0, 0.0};
  Eigen::Vector3d angular_velocity{0.0, 0.0, 0.0};
};

struct CoreAcceleration
{
  Eigen::Vector3d linear{0.0, 0.0, 0.0};
};

struct CoreInputFrame
{
  double stamp_sec{0.0};
  std::vector<cv::Mat> camera_images;
  std::vector<CoreCameraInfo> camera_infos;
  std::optional<CoreKinematicState> kinematic_state;
  std::optional<CoreAcceleration> acceleration;
};

}  // namespace autoware::tensorrt_vad

#endif  // CORE__SENSOR_TYPES_HPP_
