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

#include "../src/networks/backbone.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad
{

// Backbone class implementation

Backbone::Backbone(
  const VadConfig & vad_config,
  const autoware::tensorrt_common::TrtCommonConfig & trt_common_config,
  const std::string & plugins_path, std::shared_ptr<VadLogger> logger)
: Net(vad_config, trt_common_config, NetworkType::BACKBONE, plugins_path, logger)
{
  // Initialize TensorRT engine after derived class is constructed
  trt_common = init_tensorrt(vad_config, trt_common_config, plugins_path);
  if (!trt_common) {
    logger_->error("Failed to initialize TensorRT engine for backbone");
  }
}

std::vector<autoware::tensorrt_common::NetworkIO> Backbone::setup_network_io(
  const VadConfig & vad_config)
{
  int32_t downsampled_image_height = vad_config.target_image_height / vad_config.downsample_factor;
  int32_t downsampled_image_width = vad_config.target_image_width / vad_config.downsample_factor;
  nvinfer1::Dims camera_input_dims{
    4, {vad_config.num_cameras, 3, vad_config.target_image_height, vad_config.target_image_width}};
  nvinfer1::Dims backbone_output_dims{
    5,
    {vad_config.num_cameras, 1, vad_config.bev_feature_dim, downsampled_image_height,
     downsampled_image_width}};

  std::vector<autoware::tensorrt_common::NetworkIO> backbone_network_io;
  backbone_network_io.emplace_back("img", camera_input_dims);
  backbone_network_io.emplace_back("out.0", backbone_output_dims);
  return backbone_network_io;
}

}  // namespace autoware::tensorrt_vad
