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

#include "autoware/tensorrt_vad/vad_config.hpp"

#include "autoware/tensorrt_vad/networks/postprocess/map_postprocess.hpp"
#include "autoware/tensorrt_vad/networks/postprocess/object_postprocess.hpp"

namespace autoware::tensorrt_vad
{

MultiCameraPreprocessConfig VadConfig::create_multi_camera_preprocess_config() const
{
  MultiCameraPreprocessConfig config;

  config.input_width = input_image_width;
  config.input_height = input_image_height;
  config.output_width = target_image_width;
  config.output_height = target_image_height;
  config.num_cameras = num_cameras;

  // Calculate scale factors for bilinear interpolation
  config.scale_x = static_cast<float>(input_image_width) / target_image_width;
  config.scale_y = static_cast<float>(input_image_height) / target_image_height;

  // Copy normalization parameters
  for (int32_t i = 0; i < 3; ++i) {
    config.mean[i] = image_normalization_param_mean[i];
    config.inverse_std[i] = 1.0f / image_normalization_param_std[i];
  }

  return config;
}

MapPostprocessConfig VadConfig::create_map_postprocess_config() const
{
  MapPostprocessConfig config;

  config.map_num_queries = map_num_queries;
  config.map_num_classes = map_num_classes;
  config.map_points_per_polylines = map_points_per_polylines;
  config.num_decoder_layers = num_decoder_layers;
  config.detection_range = detection_range;
  config.map_class_names = map_class_names;
  config.map_confidence_thresholds = map_confidence_thresholds;

  return config;
}

ObjectPostprocessConfig VadConfig::create_object_postprocess_config() const
{
  ObjectPostprocessConfig config;

  config.prediction_num_queries = prediction_num_queries;
  config.prediction_num_classes = prediction_num_classes;
  config.prediction_bbox_pred_dim = prediction_bbox_pred_dim;
  config.prediction_trajectory_modes = prediction_trajectory_modes;
  config.prediction_timesteps = prediction_timesteps;
  config.num_decoder_layers = num_decoder_layers;
  config.bbox_class_names = bbox_class_names;
  config.object_confidence_thresholds = object_confidence_thresholds;

  return config;
}

}  // namespace autoware::tensorrt_vad
