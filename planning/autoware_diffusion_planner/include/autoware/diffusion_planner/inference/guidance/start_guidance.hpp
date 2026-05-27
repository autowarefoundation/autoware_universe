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

#ifndef AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__START_GUIDANCE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__START_GUIDANCE_HPP_

#include "autoware/diffusion_planner/inference/guidance/guidance.hpp"

#include <vector>

namespace autoware::diffusion_planner
{

struct StartGuidanceConfig
{
  float reference_distance_m{10.0f};
  float max_scale{1.0f};
  float x_mean{0.0f};
  float y_mean{0.0f};
  float x_std{1.0f};
  float y_std{1.0f};
};

class StartGuidance final : public Guidance
{
public:
  StartGuidance();
  explicit StartGuidance(const StartGuidanceConfig & config);

  GuidanceResult compute_delta(
    const GuidanceContext & context, const std::vector<float> & model_output) const override;

private:
  StartGuidanceConfig config_;
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__START_GUIDANCE_HPP_
