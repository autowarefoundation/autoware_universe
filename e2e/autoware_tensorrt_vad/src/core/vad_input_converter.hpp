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

#ifndef CORE__VAD_INPUT_CONVERTER_HPP_
#define CORE__VAD_INPUT_CONVERTER_HPP_

#include "core/sensor_types.hpp"
#include "core/transform_provider.hpp"
#include "data_types.hpp"
#include "vad_interface_config.hpp"

#include <Eigen/Dense>

#include <optional>
#include <vector>

namespace autoware::tensorrt_vad
{

class VadInputConverter
{
public:
  VadInputConverter(const TransformProvider & transform_provider, const VadInterfaceConfig & config);

  VadInputData convert(const CoreInputFrame & frame);

private:
  const TransformProvider & transform_provider_;
  const VadInterfaceConfig & config_;
  std::vector<float> prev_can_bus_;
  std::optional<std::vector<float>> cached_vad_base2img_;

  std::vector<float> compute_vad_base2img(const std::vector<CoreCameraInfo> & camera_infos) const;
  std::vector<float> compute_can_bus(
    const CoreKinematicState & kinematic_state, const CoreAcceleration & acceleration) const;
  std::vector<float> compute_shift(
    const std::vector<float> & can_bus, const std::vector<float> & prev_can_bus) const;

  Eigen::Matrix4f create_cam2img(const CoreCameraInfo & camera_info) const;
  std::pair<float, float> calculate_scale(const CoreCameraInfo & camera_info) const;
  Eigen::Matrix4f apply_scaling(
    const Eigen::Matrix4f & vad_base2img, float scale_width, float scale_height) const;
  std::vector<float> matrix_to_flat(const Eigen::Matrix4f & matrix) const;
};

}  // namespace autoware::tensorrt_vad

#endif  // CORE__VAD_INPUT_CONVERTER_HPP_
