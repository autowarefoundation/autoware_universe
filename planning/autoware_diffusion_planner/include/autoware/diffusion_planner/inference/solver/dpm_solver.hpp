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

#ifndef AUTOWARE__DIFFUSION_PLANNER__INFERENCE__SOLVER__DPM_SOLVER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__INFERENCE__SOLVER__DPM_SOLVER_HPP_

#include "autoware/diffusion_planner/inference/guidance/guidance.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::diffusion_planner
{

class DpmSolver
{
public:
  struct SampleResult
  {
    std::vector<float> final_x;
    std::vector<std::vector<float>> denoising_steps;
    std::vector<float> denoising_timesteps;
    std::unordered_map<std::string, std::vector<bool>> guidance_triggered;
  };

  using ModelFunction =
    std::function<std::vector<float>(const std::vector<float> & x, float timestep)>;
  using CorrectingFunction = std::function<void(std::vector<float> & x)>;
  using GuidanceMap = std::unordered_map<std::string, std::shared_ptr<Guidance>>;

  explicit DpmSolver(int steps = 10);

  SampleResult sample(
    const std::vector<float> & initial_x, const ModelFunction & model_fn,
    const CorrectingFunction & correcting_fn, const GuidanceMap & guidances = {}) const;

private:
  int steps_{10};

  static float marginal_log_mean_coeff(float t);
  static float marginal_alpha(float t);
  static float marginal_std(float t);
  static float marginal_lambda(float t);
  static float inverse_lambda(float lambda);
  static std::vector<float> log_snr_timesteps(int steps);

  static std::vector<float> first_update(
    const std::vector<float> & x_s, const std::vector<float> & model_s, float s, float t);
  static std::vector<float> second_update(
    const std::vector<float> & x_s, const std::vector<std::vector<float>> & model_prev_list,
    const std::vector<float> & t_prev_list, float t);
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__INFERENCE__SOLVER__DPM_SOLVER_HPP_
