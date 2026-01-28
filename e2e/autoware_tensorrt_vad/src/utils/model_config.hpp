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

#ifndef UTILS__MODEL_CONFIG_HPP_
#define UTILS__MODEL_CONFIG_HPP_

#include <array>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad::utils
{

/**
 * @brief Model parameters loaded from vad-carla-tiny.param.json
 *
 * Contains all model-specific parameters that are determined during training
 * and should not be changed during deployment.
 */
struct ModelParams
{
  // Version information
  int major_version;       ///< Major version (incompatible changes to model I/O)
  int minor_version;       ///< Minor version (compatible weight updates)
  std::string model_name;  ///< Model identifier (e.g., "vad-carla-tiny")

  // Image normalization parameters (from training)
  std::array<float, 3> image_normalization_mean;  ///< ImageNet mean [R, G, B]
  std::array<float, 3> image_normalization_std;   ///< ImageNet std [R, G, B]

  // Input specifications (determined by model architecture)
  int target_image_width;   ///< Model input width (e.g., 640)
  int target_image_height;  ///< Model input height (e.g., 384)

  // Class definitions (from training dataset)
  std::vector<std::string> map_classes;     ///< Map element class names
  std::vector<std::string> object_classes;  ///< Object class names

  // Network architecture parameters (BEV configuration)
  int bev_height;         ///< BEV grid height
  int bev_width;          ///< BEV grid width
  int bev_feature_dim;    ///< BEV feature dimension
  int downsample_factor;  ///< Spatial downsampling factor

  // Transformer architecture
  int num_decoder_layers;  ///< Number of transformer decoder layers

  // Object prediction configuration
  int prediction_num_queries;       ///< Number of object queries
  int prediction_num_classes;       ///< Number of object classes
  int prediction_bbox_pred_dim;     ///< Bounding box prediction dimension
  int prediction_trajectory_modes;  ///< Number of trajectory modes
  int prediction_timesteps;         ///< Prediction time horizon

  // Ego planning configuration
  int planning_ego_commands;  ///< Number of ego command types
  int planning_timesteps;     ///< Planning time horizon

  // Map element detection configuration
  int map_num_queries;          ///< Number of map element queries
  int map_num_classes;          ///< Number of map element classes
  int map_points_per_polyline;  ///< Points per polyline representation

  // Vehicle state representation
  int can_bus_dim;  ///< CAN bus feature dimension
};

}  // namespace autoware::tensorrt_vad::utils

#endif  // UTILS__MODEL_CONFIG_HPP_
