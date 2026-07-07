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

#pragma once

#ifndef MPPI__SAMPLING_DISTRIBUTIONS__DIFFUSION__DIFFUSION_MODEL_HPP_
#define MPPI__SAMPLING_DISTRIBUTIONS__DIFFUSION__DIFFUSION_MODEL_HPP_

#include <mppi/sampling_distributions/diffusion/obstacle_context.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace mppi
{
namespace sampling_distributions
{
namespace diffusion
{
constexpr char kDiffusionMagic[8] = {'M', 'P', 'P', 'I', 'D', 'I', 'F', 'F'};
constexpr int kDiffusionWeightVersion = 2;
constexpr int kDiffusionTimeEmbDim = 16;

struct DiffusionModelWeights
{
  int horizon = 0;
  int control_dim = 0;
  int hidden_dim = 0;
  int context_dim = 0;
  int train_timesteps = 0;
  int inference_timesteps = 0;
  int traj_dim = 0;
  std::vector<float> W1;
  std::vector<float> b1;
  std::vector<float> W2;
  std::vector<float> b2;
  std::vector<float> W3;
  std::vector<float> b3;
  std::vector<float> betas;
  std::vector<float> alpha_bar;
};

inline std::vector<float> readFloatArray(std::ifstream & in)
{
  std::uint32_t n = 0;
  in.read(reinterpret_cast<char *>(&n), sizeof(n));
  if (!in) {
    throw std::runtime_error("truncated diffusion weight file");
  }
  std::vector<float> out(n);
  in.read(reinterpret_cast<char *>(out.data()), sizeof(float) * n);
  if (!in) {
    throw std::runtime_error("truncated diffusion weight payload");
  }
  return out;
}

inline DiffusionModelWeights loadDiffusionModelWeights(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("failed to open diffusion weights: " + path);
  }
  char magic[8];
  in.read(magic, 8);
  if (!in || std::string(magic, 8) != std::string(kDiffusionMagic, 8)) {
    throw std::runtime_error("invalid diffusion weight magic in " + path);
  }
  std::uint32_t header[8];
  in.read(reinterpret_cast<char *>(header), sizeof(header));
  if (!in) {
    throw std::runtime_error("truncated diffusion header");
  }
  const int version = static_cast<int>(header[0]);
  if (version < 1 || version > kDiffusionWeightVersion) {
    throw std::runtime_error("unsupported diffusion weight version");
  }
  DiffusionModelWeights w;
  w.horizon = static_cast<int>(header[1]);
  w.control_dim = static_cast<int>(header[2]);
  w.hidden_dim = static_cast<int>(header[3]);
  w.context_dim = version >= 2 ? static_cast<int>(header[4]) : 0;
  w.train_timesteps = static_cast<int>(header[version >= 2 ? 5 : 4]);
  w.inference_timesteps = static_cast<int>(header[version >= 2 ? 6 : 5]);
  w.traj_dim = w.horizon * w.control_dim;
  w.W1 = readFloatArray(in);
  w.b1 = readFloatArray(in);
  w.W2 = readFloatArray(in);
  w.b2 = readFloatArray(in);
  w.W3 = readFloatArray(in);
  w.b3 = readFloatArray(in);
  w.betas = readFloatArray(in);
  w.alpha_bar = readFloatArray(in);
  return w;
}

inline void sinusoidalTimeEmbedding(
  const int t, const int train_timesteps, float * out, const int dim)
{
  const float tn = static_cast<float>(t) / static_cast<float>(std::max(train_timesteps - 1, 1));
  const int half = dim / 2;
  for (int i = 0; i < half; ++i) {
    const float freq =
      std::exp(-std::log(10000.0F) * static_cast<float>(i) / static_cast<float>(half));
    const float arg = tn * freq;
    out[i] = std::sin(arg);
    out[i + half] = std::cos(arg);
  }
}

inline void linearReluMatVec(
  const std::vector<float> & W, const std::vector<float> & b, const float * x, float * y,
  const int out_dim, const int in_dim)
{
  for (int o = 0; o < out_dim; ++o) {
    float acc = b[static_cast<size_t>(o)];
    const float * row = &W[static_cast<size_t>(o * in_dim)];
    for (int i = 0; i < in_dim; ++i) {
      acc += row[i] * x[i];
    }
    y[o] = acc > 0.0F ? acc : 0.0F;
  }
}

inline void linearMatVec(
  const std::vector<float> & W, const std::vector<float> & b, const float * x, float * y,
  const int out_dim, const int in_dim)
{
  for (int o = 0; o < out_dim; ++o) {
    float acc = b[static_cast<size_t>(o)];
    const float * row = &W[static_cast<size_t>(o * in_dim)];
    for (int i = 0; i < in_dim; ++i) {
      acc += row[i] * x[i];
    }
    y[o] = acc;
  }
}

inline void diffusionPredictEps(
  const DiffusionModelWeights & w, const float * x_t, const float * u_nom, const float * context,
  const int t, float * eps_out)
{
  const int h = w.hidden_dim;
  const int traj = w.traj_dim;
  const int ctx = w.context_dim;
  const int in_dim = 2 * traj + ctx + kDiffusionTimeEmbDim;
  std::vector<float> input(static_cast<size_t>(in_dim));
  std::vector<float> h1(static_cast<size_t>(h));
  std::vector<float> h2(static_cast<size_t>(h));
  float t_emb[kDiffusionTimeEmbDim];
  sinusoidalTimeEmbedding(t, w.train_timesteps, t_emb, kDiffusionTimeEmbDim);
  for (int i = 0; i < traj; ++i) {
    input[static_cast<size_t>(i)] = x_t[i];
    input[static_cast<size_t>(traj + i)] = u_nom[i];
  }
  for (int i = 0; i < ctx; ++i) {
    input[static_cast<size_t>(2 * traj + i)] = context != nullptr ? context[i] : 0.0F;
  }
  for (int i = 0; i < kDiffusionTimeEmbDim; ++i) {
    input[static_cast<size_t>(2 * traj + ctx + i)] = t_emb[i];
  }
  linearReluMatVec(w.W1, w.b1, input.data(), h1.data(), h, in_dim);
  linearReluMatVec(w.W2, w.b2, h1.data(), h2.data(), h, h);
  linearMatVec(w.W3, w.b3, h2.data(), eps_out, traj, h);
}

inline void diffusionSampleTrajectory(
  const DiffusionModelWeights & w, const float * u_nom, const float * context, float * u_out,
  std::mt19937 & rng)
{
  const int traj = w.traj_dim;
  std::vector<float> x(static_cast<size_t>(traj));
  std::vector<float> eps(static_cast<size_t>(traj));
  std::normal_distribution<float> normal(0.0F, 1.0F);
  for (int i = 0; i < traj; ++i) {
    x[static_cast<size_t>(i)] = normal(rng);
  }

  const int infer = std::max(1, w.inference_timesteps);
  for (int step = 0; step < infer; ++step) {
    int t = 0;
    if (infer == 1) {
      t = std::max(0, w.train_timesteps - 1);
    } else {
      const int t_val = static_cast<int>(std::lround(
        static_cast<float>(w.train_timesteps - 1) *
        (1.0F - static_cast<float>(step) / static_cast<float>(infer - 1))));
      t = std::max(0, std::min(w.train_timesteps - 1, t_val));
    }
    diffusionPredictEps(w, x.data(), u_nom, context, t, eps.data());
    const float alpha = 1.0F - w.betas[static_cast<size_t>(t)];
    const float alpha_bar = w.alpha_bar[static_cast<size_t>(t)];
    const float beta = w.betas[static_cast<size_t>(t)];
    const float inv_sqrt_alpha = 1.0F / std::sqrt(std::max(alpha, 1.0E-8F));
    const float coef2 = beta / std::sqrt(std::max(1.0F - alpha_bar, 1.0E-8F));
    for (int i = 0; i < traj; ++i) {
      x[static_cast<size_t>(i)] =
        inv_sqrt_alpha * (x[static_cast<size_t>(i)] - coef2 * eps[static_cast<size_t>(i)]);
    }
    if (step + 1 < infer) {
      const float noise_scale = std::sqrt(std::max(beta, 0.0F));
      for (int i = 0; i < traj; ++i) {
        x[static_cast<size_t>(i)] += noise_scale * normal(rng);
      }
    }
  }

  for (int i = 0; i < traj; ++i) {
    u_out[i] = u_nom[i] + x[static_cast<size_t>(i)];
  }
}

}  // namespace diffusion
}  // namespace sampling_distributions
}  // namespace mppi

#endif  // MPPI__SAMPLING_DISTRIBUTIONS__DIFFUSION__DIFFUSION_MODEL_HPP_
