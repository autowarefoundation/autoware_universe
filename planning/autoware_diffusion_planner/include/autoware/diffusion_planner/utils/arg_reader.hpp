// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__ARG_READER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__ARG_READER_HPP_

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
namespace autoware::diffusion_planner::utils
{
using json = nlohmann::json;

// Define normalization structure: {name -> (mean, std)}
using NormalizationMap =
  std::unordered_map<std::string, std::pair<std::vector<float>, std::vector<float>>>;

inline NormalizationMap load_normalization_stats(const std::string & json_path)
{
  std::ifstream file(json_path);
  if (!file) {
    throw std::runtime_error("Could not open JSON file: " + json_path);
  }

  json j;
  file >> j;

  if (!j.contains("observation_normalizer")) {
    throw std::runtime_error("Missing 'observation_normalizer' key in JSON.");
  }

  NormalizationMap norm_map;

  for (const auto & [key, val] : j["observation_normalizer"].items()) {
    std::vector<float> mean;
    std::vector<float> std_dev;
    if (val.contains("mean")) {
      for (const auto & v : val["mean"]) {
        mean.push_back(v.get<float>());
      }
    }
    if (val.contains("std")) {
      for (const auto & v : val["std"]) {
        std_dev.push_back(v.get<float>());
      }
    }
    norm_map[key] = std::make_pair(mean, std_dev);
  }

  return norm_map;
}
}  // namespace autoware::diffusion_planner::utils
#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__ARG_READER_HPP_
