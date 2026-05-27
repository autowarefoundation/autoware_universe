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

#ifndef AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__GUIDANCE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__GUIDANCE_HPP_

#include <string>
#include <vector>

namespace autoware::diffusion_planner
{

struct GuidanceContext
{
  const std::vector<float> & x;
  float timestep;
  float alpha;
  float sigma;
};

struct GuidanceResult
{
  std::vector<float> delta;
  std::vector<bool> triggered;
};

class Guidance
{
public:
  virtual ~Guidance() = default;

  Guidance() = default;
  Guidance(const Guidance &) = default;
  Guidance & operator=(const Guidance &) = default;
  Guidance(Guidance &&) = default;
  Guidance & operator=(Guidance &&) = default;

  void set_enabled(bool enabled);
  bool is_enabled() const;

  virtual GuidanceResult compute_delta(
    const GuidanceContext & context, const std::vector<float> & model_output) const = 0;

private:
  bool enabled_{false};
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__INFERENCE__GUIDANCE__GUIDANCE_HPP_
