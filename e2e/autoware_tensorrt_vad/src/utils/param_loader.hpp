// Copyright 2025 TIER IV.
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

#ifndef UTILS__PARAM_LOADER_HPP_
#define UTILS__PARAM_LOADER_HPP_

#include "utils/model_config.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>
#include <string>

namespace autoware::tensorrt_vad::utils
{
using json = nlohmann::json;

/**
 * @brief Loads model parameters from JSON configuration file
 *
 * Parses the vad-carla-tiny.param.json file and extracts all model-specific
 * parameters including normalization statistics, class definitions, and
 * network architecture parameters.
 *
 * @param json_path Path to the model parameter JSON file
 * @return ModelParams struct containing all loaded parameters
 * @throws std::runtime_error if file cannot be opened or parsed
 */
inline ModelParams load_model_params(const std::string & json_path)
{
  std::ifstream file(json_path);
  if (!file) {
    throw std::runtime_error("Could not open param.json file: " + json_path);
  }

  json j;
  file >> j;

  ModelParams params;

  // Version information
  params.major_version = j["major_version"].get<int>();
  params.minor_version = j["minor_version"].get<int>();
  params.model_name = j["model_name"].get<std::string>();

  // Image normalization parameters
  auto norm = j["image_normalization"];
  for (size_t i = 0; i < 3; ++i) {
    params.image_normalization_mean[i] = norm["mean"][i].get<float>();
    params.image_normalization_std[i] = norm["std"][i].get<float>();
  }

  // Input specifications
  auto input_specs = j["input_specs"];
  params.target_image_width = input_specs["target_image_width"].get<int>();
  params.target_image_height = input_specs["target_image_height"].get<int>();

  // Class definitions
  auto classes = j["class_definitions"];
  for (const auto & cls : classes["map_classes"]) {
    params.map_classes.push_back(cls.get<std::string>());
  }
  for (const auto & cls : classes["object_classes"]) {
    params.object_classes.push_back(cls.get<std::string>());
  }

  // Network architecture parameters
  auto arch = j["network_architecture"];

  // BEV configuration
  params.bev_height = arch["bev"]["height"].get<int>();
  params.bev_width = arch["bev"]["width"].get<int>();
  params.bev_feature_dim = arch["bev"]["feature_dim"].get<int>();
  params.downsample_factor = arch["bev"]["downsample_factor"].get<int>();

  // Transformer configuration
  params.num_decoder_layers = arch["transformer"]["num_decoder_layers"].get<int>();

  // Object prediction configuration
  params.prediction_num_queries = arch["prediction"]["num_queries"].get<int>();
  params.prediction_num_classes = arch["prediction"]["num_classes"].get<int>();
  params.prediction_bbox_pred_dim = arch["prediction"]["bbox_pred_dim"].get<int>();
  params.prediction_trajectory_modes = arch["prediction"]["trajectory_modes"].get<int>();
  params.prediction_timesteps = arch["prediction"]["timesteps"].get<int>();

  // Ego planning configuration
  params.planning_ego_commands = arch["planning"]["ego_commands"].get<int>();
  params.planning_timesteps = arch["planning"]["timesteps"].get<int>();

  // Map element detection configuration
  params.map_num_queries = arch["map"]["num_queries"].get<int>();
  params.map_num_classes = arch["map"]["num_classes"].get<int>();
  params.map_points_per_polyline = arch["map"]["points_per_polyline"].get<int>();

  // Vehicle state configuration
  params.can_bus_dim = arch["vehicle_state"]["can_bus_dim"].get<int>();

  return params;
}

}  // namespace autoware::tensorrt_vad::utils

#endif  // UTILS__PARAM_LOADER_HPP_
