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

#include "autoware/diffusion_planner/inference/solver/dpm_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
namespace
{
constexpr int k_dpm_solver_order = 2;
constexpr float k_noise_schedule_t = 1.0f;
constexpr float k_noise_schedule_total_n = 1000.0f;
constexpr float k_noise_schedule_beta0 = 0.1f;
constexpr float k_noise_schedule_beta1 = 20.0f;

float log_add_exp(float a, float b)
{
  const float m = std::max(a, b);
  return m + std::log(std::exp(a - m) + std::exp(b - m));
}
}  // namespace

DpmSolver::DpmSolver(int steps) : steps_(steps)
{
  if (steps_ < k_dpm_solver_order) {
    throw std::invalid_argument("DpmSolver steps must be greater than or equal to solver order.");
  }
}

float DpmSolver::marginal_log_mean_coeff(float t)
{
  return -0.25f * t * t * (k_noise_schedule_beta1 - k_noise_schedule_beta0) -
         0.5f * t * k_noise_schedule_beta0;
}

float DpmSolver::marginal_alpha(float t)
{
  return std::exp(marginal_log_mean_coeff(t));
}

float DpmSolver::marginal_std(float t)
{
  return std::sqrt(1.0f - std::exp(2.0f * marginal_log_mean_coeff(t)));
}

float DpmSolver::marginal_lambda(float t)
{
  const float log_mean_coeff = marginal_log_mean_coeff(t);
  const float log_std = 0.5f * std::log(1.0f - std::exp(2.0f * log_mean_coeff));
  return log_mean_coeff - log_std;
}

float DpmSolver::inverse_lambda(float lambda)
{
  const float beta_delta = k_noise_schedule_beta1 - k_noise_schedule_beta0;
  const float tmp = 2.0f * beta_delta * log_add_exp(-2.0f * lambda, 0.0f);
  const float delta = k_noise_schedule_beta0 * k_noise_schedule_beta0 + tmp;
  return tmp / (std::sqrt(delta) + k_noise_schedule_beta0) / beta_delta;
}

std::vector<float> DpmSolver::log_snr_timesteps(int steps)
{
  const float t0 = 1.0f / k_noise_schedule_total_n;
  const float lambda_t = marginal_lambda(k_noise_schedule_t);
  const float lambda_0 = marginal_lambda(t0);

  std::vector<float> timesteps;
  timesteps.reserve(static_cast<size_t>(steps) + 1);
  for (int i = 0; i <= steps; ++i) {
    const float ratio = static_cast<float>(i) / static_cast<float>(steps);
    const float lambda = lambda_t + (lambda_0 - lambda_t) * ratio;
    timesteps.push_back(inverse_lambda(lambda));
  }
  return timesteps;
}

std::vector<float> DpmSolver::first_update(
  const std::vector<float> & x_s, const std::vector<float> & model_s, float s, float t)
{
  const float lambda_s = marginal_lambda(s);
  const float lambda_t = marginal_lambda(t);
  const float h = lambda_t - lambda_s;
  const float sigma_s = marginal_std(s);
  const float sigma_t = marginal_std(t);
  const float alpha_t = marginal_alpha(t);
  const float phi_1 = std::expm1(-h);

  std::vector<float> x_t(x_s.size());
  for (size_t i = 0; i < x_s.size(); ++i) {
    x_t[i] = (sigma_t / sigma_s) * x_s[i] - alpha_t * phi_1 * model_s[i];
  }
  return x_t;
}

std::vector<float> DpmSolver::second_update(
  const std::vector<float> & x_s, const std::vector<std::vector<float>> & model_prev_list,
  const std::vector<float> & t_prev_list, float t)
{
  const auto & model_prev_1 = model_prev_list[0];
  const auto & model_prev_0 = model_prev_list[1];
  const float t_prev_1 = t_prev_list[0];
  const float t_prev_0 = t_prev_list[1];
  const float lambda_prev_1 = marginal_lambda(t_prev_1);
  const float lambda_prev_0 = marginal_lambda(t_prev_0);
  const float lambda_t = marginal_lambda(t);
  const float sigma_prev_0 = marginal_std(t_prev_0);
  const float sigma_t = marginal_std(t);
  const float alpha_t = marginal_alpha(t);
  const float h_0 = lambda_prev_0 - lambda_prev_1;
  const float h = lambda_t - lambda_prev_0;
  const float r0 = h_0 / h;
  const float phi_1 = std::expm1(-h);

  std::vector<float> x_t(x_s.size());
  for (size_t i = 0; i < x_s.size(); ++i) {
    const float d1_0 = (model_prev_0[i] - model_prev_1[i]) / r0;
    x_t[i] = (sigma_t / sigma_prev_0) * x_s[i] - (alpha_t * phi_1) * model_prev_0[i] -
             0.5f * (alpha_t * phi_1) * d1_0;
  }
  return x_t;
}

DpmSolver::SampleResult DpmSolver::sample(
  const std::vector<float> & initial_x, const ModelFunction & model_fn,
  const CorrectingFunction & correcting_fn, const GuidanceMap & guidances) const
{
  std::vector<float> x = initial_x;
  correcting_fn(x);
  SampleResult result;

  for (const auto & [name, guidance] : guidances) {
    if (guidance) {
      result.guidance_triggered[name].clear();
    }
  }

  const auto evaluate_model =
    [&](const std::vector<float> & x_input, const float timestep, bool is_last_step = false) {
      std::vector<float> model_output = model_fn(x_input, timestep);
      if (is_last_step) {
        return model_output;
      }
      const GuidanceContext context{
        x_input, timestep, marginal_alpha(timestep), marginal_std(timestep)};
      std::vector<float> total_delta(model_output.size(), 0.0f);
      for (const auto & [name, guidance] : guidances) {
        if (!guidance || !guidance->is_enabled()) {
          continue;
        }
        const GuidanceResult guidance_result = guidance->compute_delta(context, model_output);
        auto & name_triggered = result.guidance_triggered[name];
        if (guidance_result.triggered.empty()) {
          continue;
        }
        if (name_triggered.empty()) {
          name_triggered = guidance_result.triggered;
        } else {
          for (size_t i = 0; i < guidance_result.triggered.size(); ++i) {
            name_triggered[i] = name_triggered[i] || guidance_result.triggered[i];
          }
        }
        if (guidance_result.delta.empty()) {
          continue;
        }
        if (guidance_result.delta.size() != model_output.size()) {
          throw std::runtime_error(
            "Guidance delta size mismatch. expected=" + std::to_string(model_output.size()) +
            " actual=" + std::to_string(guidance_result.delta.size()));
        }
        for (size_t i = 0; i < model_output.size(); ++i) {
          total_delta[i] += guidance_result.delta[i];
        }
      }
      for (size_t i = 0; i < model_output.size(); ++i) {
        model_output[i] += total_delta[i];
      }
      return model_output;
    };

  const std::vector<float> timesteps = log_snr_timesteps(steps_);
  std::vector<float> t_prev_list{k_noise_schedule_t};
  std::vector<std::vector<float>> model_prev_list{evaluate_model(x, timesteps.front())};

  for (int step = 1; step < k_dpm_solver_order; ++step) {
    const float t = timesteps[step];
    x = first_update(x, model_prev_list.back(), t_prev_list.back(), t);
    correcting_fn(x);
    result.denoising_steps.push_back(x);
    result.denoising_timesteps.push_back(t);
    t_prev_list.push_back(t);
    model_prev_list.push_back(evaluate_model(x, t));
  }

  for (int step = k_dpm_solver_order; step <= steps_; ++step) {
    const float t = timesteps[step];
    x = second_update(x, model_prev_list, t_prev_list, t);
    correcting_fn(x);
    result.denoising_steps.push_back(x);
    result.denoising_timesteps.push_back(t);

    t_prev_list[0] = t_prev_list[1];
    t_prev_list[1] = t;
    model_prev_list[0] = std::move(model_prev_list[1]);
    if (step < steps_) {
      model_prev_list[1] = evaluate_model(x, t);
    }
  }

  x = evaluate_model(x, 1.0f / k_noise_schedule_total_n, true);
  correcting_fn(x);
  result.denoising_steps.push_back(x);
  result.denoising_timesteps.push_back(1.0f / k_noise_schedule_total_n);
  result.final_x = std::move(x);
  return result;
}

}  // namespace autoware::diffusion_planner
