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

#include "autoware/ptv3/detection3d_module.hpp"

#include <autoware/cuda_utils/cuda_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::ptv3
{

Detection3DModule::Detection3DModule(
  const tensorrt_common::TrtCommonConfig & trt_config, const PTv3Detection3DConfig & config,
  const float * backbone_point_feat, const std::int32_t * backbone_point_grid_coord)
: config_(config),
  backbone_point_feat_(backbone_point_feat),
  backbone_point_grid_coord_(backbone_point_grid_coord)
{
  const auto det_grid_size = config_.det_grid_x_size_ * config_.det_grid_y_size_;
  const auto det_class_size = config_.detection_class_names_.size();

  dense_heatmap_d_ = autoware::cuda_utils::make_unique<float[]>(det_grid_size * det_class_size);
  query_heatmap_score_d_ =
    autoware::cuda_utils::make_unique<float[]>(det_class_size * config_.num_proposals_);
  query_labels_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.num_proposals_);
  heatmap_d_ = autoware::cuda_utils::make_unique<float[]>(det_class_size * config_.num_proposals_);
  center_d_ = autoware::cuda_utils::make_unique<float[]>(2 * config_.num_proposals_);
  height_d_ = autoware::cuda_utils::make_unique<float[]>(config_.num_proposals_);
  dim_d_ = autoware::cuda_utils::make_unique<float[]>(3 * config_.num_proposals_);
  rot_d_ = autoware::cuda_utils::make_unique<float[]>(2 * config_.num_proposals_);
  if (config_.has_twist_) {
    vel_d_ = autoware::cuda_utils::make_unique<float[]>(2 * config_.num_proposals_);
  }

  post_ptr_ = std::make_unique<Detection3DPostprocess>(config_);
  initTrt(trt_config);
}

void Detection3DModule::preparePreprocess()
{
}

void Detection3DModule::setInputShapes(const std::int64_t num_voxels)
{
  trt_ptr_->setInputShape(
    "point_feat", nvinfer1::Dims{2, {num_voxels, config_.backbone_feat_dim_}});
  trt_ptr_->setInputShape("point_grid_coord", nvinfer1::Dims{2, {num_voxels, 3}});
}

bool Detection3DModule::enqueue(const PTv3ExecutionContext & context)
{
  if (!trt_ptr_->enqueueV3(context.stream())) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Fail to enqueue Detection3D head.");
    return false;
  }
  return true;
}

bool Detection3DModule::postProcess(
  const PTv3ExecutionContext & context, std::vector<Box3D> & detection_boxes)
{
  const auto stream = context.stream();
  CHECK_CUDA_ERROR(post_ptr_->process(
    query_heatmap_score_d_.get(), query_labels_d_.get(), heatmap_d_.get(), center_d_.get(),
    height_d_.get(), dim_d_.get(), rot_d_.get(), config_.has_twist_ ? vel_d_.get() : nullptr,
    stream));

  const Box3D * device_boxes = post_ptr_->deviceBoxes();
  const std::size_t num_boxes = post_ptr_->numBoxes();

  detection_boxes.resize(num_boxes);
  if (num_boxes == 0) {
    return true;
  }
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    detection_boxes.data(), device_boxes, num_boxes * sizeof(Box3D), cudaMemcpyDeviceToHost,
    stream));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
  return true;
}

void Detection3DModule::initTrt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  network_io.emplace_back(
    "point_feat", nvinfer1::Dims{2, {-1, config_.backbone_feat_dim_}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back(
    "point_grid_coord", nvinfer1::Dims{2, {-1, 3}}, nvinfer1::DataType::kINT32);

  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims;
  profile_dims.emplace_back(
    "point_feat", nvinfer1::Dims{2, {config_.voxels_num_[0], config_.backbone_feat_dim_}},
    nvinfer1::Dims{2, {config_.voxels_num_[1], config_.backbone_feat_dim_}},
    nvinfer1::Dims{2, {config_.voxels_num_[2], config_.backbone_feat_dim_}});
  profile_dims.emplace_back(
    "point_grid_coord", nvinfer1::Dims{2, {config_.voxels_num_[0], 3}},
    nvinfer1::Dims{2, {config_.voxels_num_[1], 3}}, nvinfer1::Dims{2, {config_.voxels_num_[2], 3}});

  const auto det_cls = static_cast<std::int64_t>(config_.detection_class_names_.size());
  const auto gx = static_cast<std::int64_t>(config_.det_grid_x_size_);
  const auto gy = static_cast<std::int64_t>(config_.det_grid_y_size_);
  const auto np = static_cast<std::int64_t>(config_.num_proposals_);

  network_io.emplace_back(
    "dense_heatmap", nvinfer1::Dims{4, {1, det_cls, gy, gx}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back(
    "query_heatmap_score", nvinfer1::Dims{3, {1, det_cls, np}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back("query_labels", nvinfer1::Dims{2, {1, np}}, nvinfer1::DataType::kINT64);
  network_io.emplace_back(
    "heatmap", nvinfer1::Dims{3, {1, det_cls, np}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back("center", nvinfer1::Dims{3, {1, 2, np}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back("height", nvinfer1::Dims{3, {1, 1, np}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back("dim", nvinfer1::Dims{3, {1, 3, np}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back("rot", nvinfer1::Dims{3, {1, 2, np}}, nvinfer1::DataType::kFLOAT);
  if (config_.has_twist_) {
    network_io.emplace_back("vel", nvinfer1::Dims{3, {1, 2, np}}, nvinfer1::DataType::kFLOAT);
  }

  trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
    std::vector<std::string>{config_.plugins_path_});

  if (!trt_ptr_->setup(
        std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims),
        std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io))) {
    throw std::runtime_error("Failed to setup Detection3D head TRT engine.");
  }

  trt_ptr_->setTensorAddress("point_feat", const_cast<float *>(backbone_point_feat_));
  trt_ptr_->setTensorAddress(
    "point_grid_coord", const_cast<std::int32_t *>(backbone_point_grid_coord_));
  trt_ptr_->setTensorAddress("dense_heatmap", dense_heatmap_d_.get());
  trt_ptr_->setTensorAddress("query_heatmap_score", query_heatmap_score_d_.get());
  trt_ptr_->setTensorAddress("query_labels", query_labels_d_.get());
  trt_ptr_->setTensorAddress("heatmap", heatmap_d_.get());
  trt_ptr_->setTensorAddress("center", center_d_.get());
  trt_ptr_->setTensorAddress("height", height_d_.get());
  trt_ptr_->setTensorAddress("dim", dim_d_.get());
  trt_ptr_->setTensorAddress("rot", rot_d_.get());
  if (config_.has_twist_) {
    trt_ptr_->setTensorAddress("vel", vel_d_.get());
  }
}

}  // namespace autoware::ptv3
