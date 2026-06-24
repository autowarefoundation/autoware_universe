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

#include "autoware/ptv3/backbone_preprocessor.hpp"

#include "autoware/ptv3/preprocess/point_type.hpp"

#include <autoware/cuda_utils/cuda_utils.hpp>
#include <autoware/point_types/memory.hpp>
#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ptv3
{

BackbonePreprocessor::BackbonePreprocessor(const PTv3BackbonePreprocessConfig & config)
: config_(config)
{
  grid_coord_d_ = autoware::cuda_utils::make_unique<std::int32_t[]>(config_.max_num_voxels_ * 3);
  feat_d_ = autoware::cuda_utils::make_unique<float[]>(config_.max_num_voxels_ * 4);
  serialized_code_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_ * 2);
  compact_points_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(
    config_.max_num_voxels_ * sizeof(CloudPointTypeXYZIRCAEDT));

  allocateSerializedPoolingBuffers();
}

void BackbonePreprocessor::allocateSerializedPoolingBuffers()
{
  serialized_pooling_stages_d_.clear();
  serialized_pooling_stages_d_.reserve(config_.pooling_strides_.size());
  const auto max_num_voxels = static_cast<std::size_t>(config_.max_num_voxels_);
  const auto num_orders = config_.serialization_orders_.size();

  for (std::size_t stage_index = 0; stage_index < config_.pooling_strides_.size(); ++stage_index) {
    SerializedPoolingDeviceStage stage;
    stage.indices = autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels);
    stage.indptr = autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels + 1);
    stage.head_indices = autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels);
    stage.cluster = autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels);
    stage.grid_coord = autoware::cuda_utils::make_unique<std::int32_t[]>(max_num_voxels * 3);
    stage.serialized_code =
      autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels * num_orders);
    stage.serialized_order =
      autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels * num_orders);
    stage.serialized_inverse =
      autoware::cuda_utils::make_unique<std::int64_t[]>(max_num_voxels * num_orders);
    serialized_pooling_stages_d_.push_back(std::move(stage));
  }

  serialized_pooling_num_voxels_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(config_.pooling_strides_.size() + 1);
  serialized_pooling_num_voxels_.assign(config_.pooling_strides_.size() + 1, 0);
}

std::vector<SerializedPoolingDeviceStageView> BackbonePreprocessor::serializedPoolingStageViews()
{
  std::vector<SerializedPoolingDeviceStageView> stage_views;
  stage_views.reserve(serialized_pooling_stages_d_.size());
  for (auto & stage : serialized_pooling_stages_d_) {
    stage_views.push_back(
      SerializedPoolingDeviceStageView{
        stage.indices.get(), stage.indptr.get(), stage.head_indices.get(), stage.cluster.get(),
        stage.grid_coord.get(), stage.serialized_code.get(), stage.serialized_order.get(),
        stage.serialized_inverse.get()});
  }
  return stage_views;
}

void BackbonePreprocessor::prepareCloudFormat(const cuda_blackboard::CudaPointCloud2 & msg)
{
  std::call_once(init_cloud_, [this, &msg]() {
    input_format_ = detectCloudFormat(msg);
    if (input_format_ == CloudFormat::UNKNOWN) {
      throw std::runtime_error(
        "Unsupported point cloud type. Expected one of: XYZIRCAEDT (10 fields), "
        "XYZIRADRT (9 fields), XYZIRC (6 fields), or XYZI (4 fields).");
    }

    RCLCPP_INFO(
      rclcpp::get_logger("ptv3"), "Detected input format with %zu fields and point step %zu bytes",
      get_num_fields(input_format_), get_point_step(input_format_));
  });
}

bool BackbonePreprocessor::run(
  const cuda_blackboard::CudaPointCloud2 & msg, const PTv3ExecutionContext & context)
{
  using autoware::cuda_utils::clear_async;
  const auto stream = context.stream();

  if (pre_ptr_ == nullptr) {
    pre_ptr_ = std::make_unique<PreprocessCuda>(config_);
  }

  const auto num_points = msg.height * msg.width;
  num_input_points_ = num_points;
  if (num_points == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Empty pointcloud. Skipping inference.");
    return false;
  }

  clear_async(feat_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 4, stream);
  clear_async(grid_coord_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 3, stream);
  clear_async(
    serialized_code_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 2, stream);
  clear_async(
    compact_points_d_.get(),
    static_cast<std::size_t>(config_.max_num_voxels_) * sizeof(CloudPointTypeXYZIRCAEDT), stream);

  num_voxels_ = pre_ptr_->generateFeatures(
    msg.data.get(), input_format_, num_points, feat_d_.get(), grid_coord_d_.get(),
    serialized_code_d_.get(), compact_points_d_.get(), &num_cropped_points_, stream);

  if (num_voxels_ < config_.min_num_voxels_) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ptv3"), "Too few voxels (" << num_voxels_
                                                     << ") for the actual optimization profile ("
                                                     << config_.min_num_voxels_ << ")");
    return false;
  }
  if (num_voxels_ > config_.max_num_voxels_) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("ptv3"), "Actual number of voxels ("
                                    << num_voxels_
                                    << ") is over the limit for the actual optimization profile ("
                                    << config_.max_num_voxels_ << "). Clipping to the limit.");
    num_voxels_ = config_.max_num_voxels_;
  }

  precomputeSerializedPoolingMetadata(context);
  return true;
}

void BackbonePreprocessor::precomputeSerializedPoolingMetadata(const PTv3ExecutionContext & context)
{
  if (config_.pooling_strides_.empty()) {
    return;
  }

  const auto stream = context.stream();
  pre_ptr_->generateSerializedPoolingMetadata(
    grid_coord_d_.get(), serialized_code_d_.get(), num_voxels_, serializedPoolingStageViews(),
    serialized_pooling_num_voxels_d_.get(), stream);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    serialized_pooling_num_voxels_.data(), serialized_pooling_num_voxels_d_.get(),
    serialized_pooling_num_voxels_.size() * sizeof(std::int64_t), cudaMemcpyDeviceToHost, stream));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
}

CloudFormat BackbonePreprocessor::detectCloudFormat(
  const cuda_blackboard::CudaPointCloud2 & cloud) const
{
  const auto & fields = cloud.fields;
  const auto num_fields = fields.size();

  if (num_fields == 10 && point_types::is_data_layout_compatible_with_point_xyzircaedt(fields)) {
    return CloudFormat::XYZIRCAEDT;
  }
  if (num_fields == 9 && point_types::is_data_layout_compatible_with_point_xyziradrt(fields)) {
    return CloudFormat::XYZIRADRT;
  }
  if (num_fields == 6 && point_types::is_data_layout_compatible_with_point_xyzirc(fields)) {
    return CloudFormat::XYZIRC;
  }
  if (num_fields == 4 && point_types::is_data_layout_compatible_with_point_xyzi(fields)) {
    return CloudFormat::XYZI;
  }

  return CloudFormat::UNKNOWN;
}

}  // namespace autoware::ptv3
