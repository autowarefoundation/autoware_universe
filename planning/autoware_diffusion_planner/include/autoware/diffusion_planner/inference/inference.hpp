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

#ifndef AUTOWARE__DIFFUSION_PLANNER__INFERENCE__INFERENCE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__INFERENCE__INFERENCE_HPP_

#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include <tl/expected.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{

struct InferenceOutput
{
  std::pair<std::vector<float>, std::vector<float>> outputs;
  std::vector<float> denoising_predictions;
  std::vector<float> denoising_timesteps;
  double inference_time_ms{0.0};
  bool is_denormalized{false};
  std::unordered_map<std::string, std::vector<bool>> guidance_triggered;
};

using InferenceResult = tl::expected<InferenceOutput, std::string>;

class Inference
{
public:
  virtual ~Inference() = default;

  virtual InferenceResult infer(const preprocess::InputDataMap & input_data_map) = 0;
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__INFERENCE__INFERENCE_HPP_
