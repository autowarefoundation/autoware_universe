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

#include "autoware/diffusion_planner/constants.hpp"

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
using ObservationNormalization =
  std::unordered_map<std::string, std::pair<std::vector<float>, std::vector<float>>>;

using StateNormalization = std::pair<std::vector<float>, std::vector<float>>;

inline std::vector<float> flatten_json_floats(const json & value)
{
  std::vector<float> result;
  const auto visit = [&result](const json & node, const auto & self) -> void {
    if (node.is_array()) {
      for (const auto & item : node) {
        self(item, self);
      }
      return;
    }
    result.push_back(node.get<float>());
  };
  visit(value, visit);
  return result;
}

inline void check_weight_version(const std::string & json_path)
{
  std::ifstream file(json_path);
  if (!file) {
    throw std::runtime_error("Could not open JSON file: " + json_path);
  }

  json j;
  file >> j;

  const std::string error_msg =
    "Please use the appropriate version of diffusion_planner.onnx and "
    "diffusion_planner.param.json. "
    "Refer to README.md for more details.";

  if (!j.contains("major_version")) {
    throw std::runtime_error("Missing 'major_version' key in JSON. " + error_msg);
  }

  const int major_version = j["major_version"].get<int>();
  if (major_version != autoware::diffusion_planner::constants::WEIGHT_MAJOR_VERSION) {
    throw std::runtime_error(
      "Unsupported major_version: " + std::to_string(major_version) + ". " + error_msg);
  }
}

inline ObservationNormalization load_observation_normalization(const std::string & json_path)
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

  ObservationNormalization norm_map;

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

inline StateNormalization load_state_normalization(const std::string & json_path)
{
  std::ifstream file(json_path);
  if (!file) {
    throw std::runtime_error("Could not open JSON file: " + json_path);
  }

  json j;
  file >> j;

  if (!j.contains("state_normalizer")) {
    throw std::runtime_error("Missing 'state_normalizer' key in JSON.");
  }

  std::vector<float> mean;
  std::vector<float> std_dev;
  const auto & state_normalizer = j["state_normalizer"];
  if (state_normalizer.contains("mean")) {
    mean = flatten_json_floats(state_normalizer["mean"]);
  }
  if (state_normalizer.contains("std")) {
    std_dev = flatten_json_floats(state_normalizer["std"]);
  }

  return std::make_pair(mean, std_dev);
}
}  // namespace autoware::diffusion_planner::utils
#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__ARG_READER_HPP_
