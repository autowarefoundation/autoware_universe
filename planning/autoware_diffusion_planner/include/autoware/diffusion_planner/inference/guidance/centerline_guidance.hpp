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

#ifndef AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__CENTERLINE_GUIDANCE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__CENTERLINE_GUIDANCE_HPP_

#include "autoware/diffusion_planner/inference/guidance/guidance.hpp"

#include <vector>

namespace autoware::diffusion_planner
{

struct CenterlineGuidanceConfig
{
  float start_time_s{2.0f};
  float x_mean{0.0f};
  float y_mean{0.0f};
  float x_std{1.0f};
  float y_std{1.0f};
};

class CenterlineGuidance final : public Guidance
{
public:
  CenterlineGuidance();
  explicit CenterlineGuidance(const CenterlineGuidanceConfig & config);

  void set_config(const CenterlineGuidanceConfig & config);
  void set_route_lanes(const std::vector<float> & route_lanes);

  GuidanceResult compute_delta(
    const GuidanceContext & context, const std::vector<float> & model_output) const override;

private:
  CenterlineGuidanceConfig config_;
  std::vector<float> route_lanes_;
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__CENTERLINE_GUIDANCE_HPP_
