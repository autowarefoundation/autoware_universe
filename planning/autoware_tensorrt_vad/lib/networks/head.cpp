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

#include "autoware/tensorrt_vad/networks/head.hpp"

namespace autoware::tensorrt_vad
{

// Head class implementation

Head::Head(
  const VadConfig & vad_config,
  const autoware::tensorrt_common::TrtCommonConfig & trt_common_config, NetworkType network_type,
  const std::string & plugins_path, std::shared_ptr<VadLogger> logger)
: Net(vad_config, trt_common_config, network_type, plugins_path, logger),
  network_type_(network_type)
{
  // Initialize TensorRT engine after derived class is constructed
  trt_common = init_tensorrt(vad_config, trt_common_config, plugins_path);
  if (!trt_common) {
    logger_->error("Failed to initialize TensorRT engine for head");
  }
}

std::vector<autoware::tensorrt_common::NetworkIO> Head::setup_network_io(
  const VadConfig & vad_config)
{
  int32_t downsampled_image_height = vad_config.target_image_height / vad_config.downsample_factor;
  int32_t downsampled_image_width = vad_config.target_image_width / vad_config.downsample_factor;
  nvinfer1::Dims mlvl_dims{
    5,
    {1, vad_config.num_cameras, vad_config.bev_feature_dim, downsampled_image_height,
     downsampled_image_width}};
  nvinfer1::Dims can_bus_dims{2, {1, vad_config.can_bus_dim}};
  nvinfer1::Dims lidar2img_dims{3, {vad_config.num_cameras, 4, 4}};
  nvinfer1::Dims shift_dims{2, {1, 2}};
  nvinfer1::Dims prev_bev_dims{
    3, {vad_config.bev_h * vad_config.bev_w, 1, vad_config.bev_feature_dim}};
  nvinfer1::Dims ego_fut_preds_dims{
    4, {1, vad_config.planning_ego_commands, vad_config.planning_timesteps, 2}};
  nvinfer1::Dims traj_preds_dims{
    5,
    {3, 1, vad_config.prediction_num_queries, vad_config.prediction_trajectory_modes,
     vad_config.prediction_timesteps * 2}};
  nvinfer1::Dims traj_cls_dims{
    4, {3, 1, vad_config.prediction_num_queries, vad_config.prediction_trajectory_modes}};
  nvinfer1::Dims bbox_preds_dims{
    4, {3, 1, vad_config.prediction_num_queries, vad_config.prediction_bbox_pred_dim}};
  nvinfer1::Dims all_cls_scores_dims{
    4, {3, 1, vad_config.prediction_num_queries, vad_config.prediction_num_classes}};
  nvinfer1::Dims map_all_cls_scores_dims{
    4, {3, 1, vad_config.map_num_queries, vad_config.map_num_class}};
  nvinfer1::Dims map_all_pts_preds_dims{
    5, {3, 1, vad_config.map_num_queries, vad_config.map_points_per_polylines, 2}};
  nvinfer1::Dims map_all_bbox_preds_dims{4, {3, 1, vad_config.map_num_queries, 4}};

  // Common NetworkIO configuration
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;
  network_io.emplace_back("mlvl_feats.0", mlvl_dims);
  network_io.emplace_back("img_metas.0[can_bus]", can_bus_dims);
  network_io.emplace_back("img_metas.0[lidar2img]", lidar2img_dims);
  network_io.emplace_back("img_metas.0[shift]", shift_dims);

  // Add prev_bev only for HEAD network type
  if (network_type_ == NetworkType::HEAD) {
    network_io.emplace_back("prev_bev", prev_bev_dims);
  }

  // Common output tensor configuration
  network_io.emplace_back("out.bev_embed", prev_bev_dims);
  network_io.emplace_back("out.ego_fut_preds", ego_fut_preds_dims);
  network_io.emplace_back("out.all_traj_preds", traj_preds_dims);
  network_io.emplace_back("out.all_traj_cls_scores", traj_cls_dims);
  network_io.emplace_back("out.all_bbox_preds", bbox_preds_dims);
  network_io.emplace_back("out.all_cls_scores", all_cls_scores_dims);
  network_io.emplace_back("out.map_all_cls_scores", map_all_cls_scores_dims);
  network_io.emplace_back("out.map_all_pts_preds", map_all_pts_preds_dims);
  network_io.emplace_back("out.map_all_bbox_preds", map_all_bbox_preds_dims);

  return network_io;
}

}  // namespace autoware::tensorrt_vad
