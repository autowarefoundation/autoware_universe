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

#ifndef AUTOWARE__TENSORRT_VAD__VAD_CONFIG_HPP_
#define AUTOWARE__TENSORRT_VAD__VAD_CONFIG_HPP_

#include "networks/postprocess/map_postprocess_kernel.hpp"
#include "networks/postprocess/object_postprocess_kernel.hpp"
#include "networks/preprocess/multi_camera_preprocess.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad
{

// config for Net class
struct NetConfig
{
  std::string name;
  std::map<std::string, std::map<std::string, std::string>> inputs;
};

// NetworkIO configuration parameters
struct VadConfig
{
  int32_t num_cameras;
  int32_t bev_h, bev_w;
  int32_t bev_feature_dim;
  int32_t num_decoder_layers;
  int32_t prediction_num_queries;
  int32_t prediction_num_classes;
  int32_t prediction_bbox_pred_dim;
  int32_t prediction_trajectory_modes;
  int32_t prediction_timesteps;
  int32_t planning_ego_commands;
  int32_t planning_timesteps;
  int32_t can_bus_dim;
  int32_t target_image_width;
  int32_t target_image_height;
  int32_t downsample_factor;
  int32_t map_num_queries;
  int32_t map_num_class;
  int32_t map_points_per_polylines;

  std::array<float, 6> detection_range;
  std::map<std::string, float> map_confidence_thresholds;
  std::map<std::string, float> object_confidence_thresholds;
  std::vector<std::string> map_class_names;
  std::vector<std::string> bbox_class_names;
  int32_t map_num_classes;
  std::string plugins_path;
  std::vector<NetConfig> nets_config;

  // Parameters for MultiCameraPreprocessor
  int32_t input_image_width;
  int32_t input_image_height;
  std::array<float, 3> image_normalization_param_mean;
  std::array<float, 3> image_normalization_param_std;

  // Helper method to create MultiCameraPreprocessConfig
  MultiCameraPreprocessConfig create_multi_camera_preprocess_config() const;

  // Helper method to create MapPostprocessConfig
  MapPostprocessConfig create_map_postprocess_config() const;

  // Helper method to create ObjectPostprocessConfig
  ObjectPostprocessConfig create_object_postprocess_config() const;
};

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE__TENSORRT_VAD__VAD_CONFIG_HPP_
