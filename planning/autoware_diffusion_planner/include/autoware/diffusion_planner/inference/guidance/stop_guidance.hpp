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

#ifndef AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__STOP_GUIDANCE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__STOP_GUIDANCE_HPP_

#include "autoware/diffusion_planner/inference/guidance/guidance.hpp"

#include <vector>

namespace autoware::diffusion_planner
{

struct StopGuidanceConfig
{
  float stop_acceleration_mps2{1.0f};
  float x_mean{0.0f};
  float y_mean{0.0f};
  float x_std{1.0f};
  float y_std{1.0f};
};

class StopGuidance final : public Guidance
{
public:
  StopGuidance();
  explicit StopGuidance(const StopGuidanceConfig & config);

  void set_config(const StopGuidanceConfig & config);
  void set_current_speed_mps(float current_speed_mps);

  GuidanceResult compute_delta(
    const GuidanceContext & context, const std::vector<float> & model_output) const override;

private:
  StopGuidanceConfig config_;
  float current_speed_mps_{0.0f};
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__STOP_GUIDANCE_HPP_
