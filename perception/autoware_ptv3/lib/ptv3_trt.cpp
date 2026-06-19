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

#include "autoware/ptv3/ptv3_trt.hpp"

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_utils.hpp>
#include <autoware/point_types/memory.hpp>
#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ptv3
{
namespace
{

std::int64_t poolingDepth(const std::int64_t stride)
{
  std::int64_t depth = 0;
  for (auto value = stride; value > 1; value >>= 1) {
    ++depth;
  }
  return depth;
}

}  // namespace

PTv3TRT::PTv3TRT(
  const tensorrt_common::TrtCommonConfig & backbone_trt_config,
  const std::optional<tensorrt_common::TrtCommonConfig> & seg3d_head_trt_config,
  const PTv3Config & config)
: config_(config)
{
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing/inner");

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  createPointFields();
  initPtr();
  initBackboneTrt(backbone_trt_config);
  if (config_.use_seg3d_head_) {
    if (!seg3d_head_trt_config.has_value()) {
      throw std::runtime_error("seg3d_head_trt_config is required when segmentation3d.use_head.");
    }
    initSeg3dHeadTrt(*seg3d_head_trt_config);
  }
}

void PTv3TRT::setPublishSegmentedPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_segmented_pointcloud_ = std::move(func);
}

void PTv3TRT::setPublishVisualizationPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_visualization_pointcloud_ = std::move(func);
}

void PTv3TRT::setPublishFilteredPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_filtered_pointcloud_ = std::move(func);
}

void PTv3TRT::allocateMessages()
{
  const auto output_capacity = config_.source_reconstruction_ != SourceReconstruction::NONE
                                 ? config_.cloud_capacity_
                                 : config_.max_num_voxels_;
  if (segmented_points_msg_ptr_ == nullptr) {
    segmented_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    segmented_points_msg_ptr_->height = 1;
    segmented_points_msg_ptr_->width = output_capacity;
    segmented_points_msg_ptr_->fields = segmented_pointcloud_fields_;
    segmented_points_msg_ptr_->is_bigendian = false;
    segmented_points_msg_ptr_->is_dense = true;
    segmented_points_msg_ptr_->point_step = 21U;
    segmented_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      output_capacity * segmented_points_msg_ptr_->point_step);
  }

  if (visualization_points_msg_ptr_ == nullptr) {
    visualization_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    visualization_points_msg_ptr_->height = 1;
    visualization_points_msg_ptr_->width = output_capacity;
    visualization_points_msg_ptr_->fields = visualization_pointcloud_fields_;
    visualization_points_msg_ptr_->is_bigendian = false;
    visualization_points_msg_ptr_->is_dense = true;
    visualization_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(visualization_pointcloud_fields_.size() * sizeof(float));
    visualization_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      output_capacity * visualization_points_msg_ptr_->point_step);
  }

  if (filtered_points_msg_ptr_ == nullptr && filtered_output_format_ != CloudFormat::UNKNOWN) {
    filtered_points_msg_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    filtered_points_msg_ptr_->height = 1;
    filtered_points_msg_ptr_->width = output_capacity;
    filtered_points_msg_ptr_->fields = filtered_pointcloud_fields_;
    filtered_points_msg_ptr_->is_bigendian = false;
    filtered_points_msg_ptr_->is_dense = true;
    filtered_points_msg_ptr_->point_step =
      static_cast<std::uint32_t>(get_point_step(filtered_output_format_));
    filtered_points_msg_ptr_->data = cuda_blackboard::make_unique<std::uint8_t[]>(
      output_capacity * filtered_points_msg_ptr_->point_step);
  }
}

PTv3TRT::~PTv3TRT()
{
  if (stream_) {
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    CHECK_CUDA_ERROR(cudaStreamDestroy(stream_));
  }
}

void PTv3TRT::initPtr()
{
  grid_coord_d_ = autoware::cuda_utils::make_unique<std::int32_t[]>(config_.max_num_voxels_ * 3);
  feat_d_ = autoware::cuda_utils::make_unique<float[]>(config_.max_num_voxels_ * 4);
  serialized_code_d_ =
    autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.max_num_voxels_ * 2);

  // Backbone outputs shared with the segmentation head.
  bb_point_feat_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.max_num_voxels_ * config_.backbone_feat_dim_);
  bb_point_grid_coord_d_ =
    autoware::cuda_utils::make_unique<std::int32_t[]>(config_.max_num_voxels_ * 3);
  bb_point_offset_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(1);

  pred_labels_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
  pred_probs_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.max_num_voxels_ * config_.class_names_.size());
  compact_points_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(
    config_.max_num_voxels_ * sizeof(CloudPointTypeXYZIRCAEDT));
  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    cropped_source_points_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(
      config_.cloud_capacity_ * sizeof(CloudPointTypeXYZIRCAEDT));
  }
  if (config_.source_reconstruction_ != SourceReconstruction::NONE) {
    reconstructed_features_d_ = autoware::cuda_utils::make_unique<float[]>(
      config_.cloud_capacity_ * config_.num_point_feature_size_);
    inverse_map_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.cloud_capacity_);
    reconstructed_labels_d_ =
      autoware::cuda_utils::make_unique<std::int64_t[]>(config_.cloud_capacity_);
    reconstructed_probs_d_ = autoware::cuda_utils::make_unique<float[]>(
      config_.cloud_capacity_ * config_.class_names_.size());
  }

  pre_ptr_ = std::make_unique<PreprocessCuda>(config_, stream_);
  post_ptr_ = std::make_unique<PostprocessCuda>(config_, stream_);

  allocateMessages();
  allocateSerializedPoolingBuffers();
}

void PTv3TRT::allocateSerializedPoolingBuffers()
{
  serialized_pooling_stages_d_.clear();
  serialized_pooling_stages_d_.reserve(config_.pooling_strides_.size());
  const auto max_num_voxels = static_cast<std::size_t>(config_.max_num_voxels_);
  const auto num_orders = config_.serialization_orders_.size();

  for (std::size_t stage_index = 0; stage_index < config_.pooling_strides_.size(); ++stage_index) {
    SerializedPoolingDeviceStage stage;
    stage.indices = autoware::cuda_utils::make_unique<std::uint32_t[]>(max_num_voxels);
    stage.indptr = autoware::cuda_utils::make_unique<std::uint32_t[]>(max_num_voxels + 1);
    stage.head_indices = autoware::cuda_utils::make_unique<std::uint32_t[]>(max_num_voxels);
    stage.cluster = autoware::cuda_utils::make_unique<std::uint32_t[]>(max_num_voxels);
    stage.grid_coord = autoware::cuda_utils::make_unique<std::int32_t[]>(max_num_voxels * 3);
    stage.serialized_code =
      autoware::cuda_utils::make_unique<std::uint32_t[]>(max_num_voxels * num_orders);
    stage.serialized_order =
      autoware::cuda_utils::make_unique<std::uint32_t[]>(max_num_voxels * num_orders);
    stage.serialized_inverse =
      autoware::cuda_utils::make_unique<std::uint32_t[]>(max_num_voxels * num_orders);
    serialized_pooling_stages_d_.push_back(std::move(stage));
  }

  serialized_pooling_num_voxels_d_ =
    autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.pooling_strides_.size() + 1);
  serialized_pooling_num_voxels_.assign(config_.pooling_strides_.size() + 1, 0);
  serialized_pooling_depths_.resize(config_.pooling_strides_.size());
  for (std::size_t stage_index = 0; stage_index < config_.pooling_strides_.size(); ++stage_index) {
    serialized_pooling_depths_[stage_index] = poolingDepth(config_.pooling_strides_[stage_index]);
  }
}

void PTv3TRT::createPointFields()
{
  auto make_point_field = [](const std::string & name, int offset, int datatype, int count) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
  };

  segmented_pointcloud_fields_.push_back(
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  segmented_pointcloud_fields_.push_back(
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  segmented_pointcloud_fields_.push_back(
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  segmented_pointcloud_fields_.push_back(
    make_point_field("class_id", 12, sensor_msgs::msg::PointField::UINT8, 1));
  segmented_pointcloud_fields_.push_back(
    make_point_field("probability", 13, sensor_msgs::msg::PointField::FLOAT32, 1));
  segmented_pointcloud_fields_.push_back(
    make_point_field("entropy", 17, sensor_msgs::msg::PointField::FLOAT32, 1));

  visualization_pointcloud_fields_.push_back(
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  visualization_pointcloud_fields_.push_back(
    make_point_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32, 1));
}

void PTv3TRT::initBackboneTrt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  // Inputs
  network_io.emplace_back("grid_coord", nvinfer1::Dims{2, {-1, 3}}, nvinfer1::DataType::kINT32);
  network_io.emplace_back("feat", nvinfer1::Dims{2, {-1, 4}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back(
    "serialized_code", nvinfer1::Dims{2, {2, -1}}, nvinfer1::DataType::kINT32);

  // Outputs: point_feat [N, backbone_feat_dim], point_grid_coord [N, 3], point_offset [1]
  network_io.emplace_back(
    "point_feat", nvinfer1::Dims{2, {-1, config_.backbone_feat_dim_}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back(
    "point_grid_coord", nvinfer1::Dims{2, {-1, 3}}, nvinfer1::DataType::kINT32);
  network_io.emplace_back("point_offset", nvinfer1::Dims{1, {1}}, nvinfer1::DataType::kINT64);

  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims;

  profile_dims.emplace_back(
    "grid_coord", nvinfer1::Dims{2, {config_.voxels_num_[0], 3}},
    nvinfer1::Dims{2, {config_.voxels_num_[1], 3}}, nvinfer1::Dims{2, {config_.voxels_num_[2], 3}});

  profile_dims.emplace_back(
    "feat", nvinfer1::Dims{2, {config_.voxels_num_[0], 4}},
    nvinfer1::Dims{2, {config_.voxels_num_[1], 4}}, nvinfer1::Dims{2, {config_.voxels_num_[2], 4}});

  profile_dims.emplace_back(
    "serialized_code", nvinfer1::Dims{2, {2, config_.voxels_num_[0]}},
    nvinfer1::Dims{2, {2, config_.voxels_num_[1]}}, nvinfer1::Dims{2, {2, config_.voxels_num_[2]}});

  // Serialized pooling metadata inputs are precomputed on device each frame and fed to the engine.
  // In the exported ONNX, indices drive native Gather, indptr drives SegmentCSR, and the remaining
  // per-stage tensors feed the following PTv3 serialization steps. Their extents are
  // data-dependent, so they are declared dynamic and bounded by the voxel-count optimization
  // profile. A pooled (output) count is at most its input count, so all pooled dims are
  // conservatively bounded by [1, opt, max] voxels.
  const auto add_pooling_io = [&network_io, &profile_dims](
                                const std::string & name, const nvinfer1::Dims & io_dims,
                                const nvinfer1::Dims & min_dims, const nvinfer1::Dims & opt_dims,
                                const nvinfer1::Dims & max_dims,
                                const std::optional<nvinfer1::DataType> data_type = std::nullopt) {
    network_io.emplace_back(name, io_dims, data_type);
    profile_dims.emplace_back(name, min_dims, opt_dims, max_dims);
  };

  const std::int64_t min_voxels = config_.voxels_num_[0];
  const std::int64_t opt_voxels = config_.voxels_num_[1];
  const std::int64_t max_voxels = config_.voxels_num_[2];
  const std::int64_t num_orders = static_cast<std::int64_t>(config_.serialization_orders_.size());

  for (std::size_t stage = 0; stage < config_.pooling_strides_.size(); ++stage) {
    const auto prefix = "serialized_pooling_" + std::to_string(stage) + "_";
    // Input-count-sized tensors. Stage 0 consumes the original voxels and therefore shares their
    // lower bound; deeper stages consume an already-pooled (smaller) count.
    const std::int64_t in_min = stage == 0 ? min_voxels : 1;
    add_pooling_io(
      prefix + "indices", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {in_min}},
      nvinfer1::Dims{1, {opt_voxels}}, nvinfer1::Dims{1, {max_voxels}}, nvinfer1::DataType::kINT32);
    add_pooling_io(
      prefix + "cluster", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {in_min}},
      nvinfer1::Dims{1, {opt_voxels}}, nvinfer1::Dims{1, {max_voxels}}, nvinfer1::DataType::kINT32);
    // Output-count-sized (pooled) tensors.
    add_pooling_io(
      prefix + "indptr", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {2}},
      nvinfer1::Dims{1, {opt_voxels + 1}}, nvinfer1::Dims{1, {max_voxels + 1}},
      nvinfer1::DataType::kINT32);
    add_pooling_io(
      prefix + "head_indices", nvinfer1::Dims{1, {-1}}, nvinfer1::Dims{1, {1}},
      nvinfer1::Dims{1, {opt_voxels}}, nvinfer1::Dims{1, {max_voxels}}, nvinfer1::DataType::kINT32);
    add_pooling_io(
      prefix + "grid_coord", nvinfer1::Dims{2, {-1, 3}}, nvinfer1::Dims{2, {1, 3}},
      nvinfer1::Dims{2, {opt_voxels, 3}}, nvinfer1::Dims{2, {max_voxels, 3}},
      nvinfer1::DataType::kINT32);
    add_pooling_io(
      prefix + "serialized_order", nvinfer1::Dims{2, {num_orders, -1}},
      nvinfer1::Dims{2, {num_orders, 1}}, nvinfer1::Dims{2, {num_orders, opt_voxels}},
      nvinfer1::Dims{2, {num_orders, max_voxels}}, nvinfer1::DataType::kINT32);
    add_pooling_io(
      prefix + "serialized_inverse", nvinfer1::Dims{2, {num_orders, -1}},
      nvinfer1::Dims{2, {num_orders, 1}}, nvinfer1::Dims{2, {num_orders, opt_voxels}},
      nvinfer1::Dims{2, {num_orders, max_voxels}}, nvinfer1::DataType::kINT32);
  }

  backbone_trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
    std::vector<std::string>{config_.plugins_path_});

  if (!backbone_trt_ptr_->setup(
        std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims),
        std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io))) {
    throw std::runtime_error("Failed to setup backbone TRT engine.");
  }

  backbone_trt_ptr_->setTensorAddress("grid_coord", grid_coord_d_.get());
  backbone_trt_ptr_->setTensorAddress("feat", feat_d_.get());
  backbone_trt_ptr_->setTensorAddress("serialized_code", serialized_code_d_.get());
  backbone_trt_ptr_->setTensorAddress("point_feat", bb_point_feat_d_.get());
  backbone_trt_ptr_->setTensorAddress("point_grid_coord", bb_point_grid_coord_d_.get());
  backbone_trt_ptr_->setTensorAddress("point_offset", bb_point_offset_d_.get());
  bindSerializedPoolingAddresses();
}

void PTv3TRT::initSeg3dHeadTrt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  network_io.emplace_back(
    "point_feat", nvinfer1::Dims{2, {-1, config_.backbone_feat_dim_}}, nvinfer1::DataType::kFLOAT);
  network_io.emplace_back("pred_labels", nvinfer1::Dims{1, {-1}}, nvinfer1::DataType::kINT64);
  network_io.emplace_back(
    "pred_probs", nvinfer1::Dims{2, {-1, static_cast<std::int64_t>(config_.class_names_.size())}},
    nvinfer1::DataType::kFLOAT);

  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims;
  profile_dims.emplace_back(
    "point_feat", nvinfer1::Dims{2, {config_.voxels_num_[0], config_.backbone_feat_dim_}},
    nvinfer1::Dims{2, {config_.voxels_num_[1], config_.backbone_feat_dim_}},
    nvinfer1::Dims{2, {config_.voxels_num_[2], config_.backbone_feat_dim_}});

  seg3d_head_trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
    std::vector<std::string>{config_.plugins_path_});

  if (!seg3d_head_trt_ptr_->setup(
        std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims),
        std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io))) {
    throw std::runtime_error("Failed to setup seg3d_head TRT engine.");
  }

  seg3d_head_trt_ptr_->setTensorAddress("point_feat", bb_point_feat_d_.get());
  seg3d_head_trt_ptr_->setTensorAddress("pred_labels", pred_labels_d_.get());
  seg3d_head_trt_ptr_->setTensorAddress("pred_probs", pred_probs_d_.get());
}

void PTv3TRT::bindSerializedPoolingAddresses()
{
  // Metadata buffers are allocated once in allocateSerializedPoolingBuffers and never reallocated,
  // so their device addresses are stable and can be bound a single time. The per-stage
  // serialized_code buffers are only used to chain pooling stages on the host side and are not
  // engine inputs, so they are intentionally not bound here.
  for (std::size_t stage = 0; stage < serialized_pooling_stages_d_.size(); ++stage) {
    const auto prefix = "serialized_pooling_" + std::to_string(stage) + "_";
    auto & buffers = serialized_pooling_stages_d_[stage];
    backbone_trt_ptr_->setTensorAddress((prefix + "indices").c_str(), buffers.indices.get());
    backbone_trt_ptr_->setTensorAddress((prefix + "indptr").c_str(), buffers.indptr.get());
    backbone_trt_ptr_->setTensorAddress(
      (prefix + "head_indices").c_str(), buffers.head_indices.get());
    backbone_trt_ptr_->setTensorAddress((prefix + "cluster").c_str(), buffers.cluster.get());
    backbone_trt_ptr_->setTensorAddress((prefix + "grid_coord").c_str(), buffers.grid_coord.get());
    backbone_trt_ptr_->setTensorAddress(
      (prefix + "serialized_order").c_str(), buffers.serialized_order.get());
    backbone_trt_ptr_->setTensorAddress(
      (prefix + "serialized_inverse").c_str(), buffers.serialized_inverse.get());
  }
}

void PTv3TRT::precomputeSerializedPoolingMetadata()
{
  if (config_.pooling_strides_.empty()) {
    return;
  }

  std::vector<SerializedPoolingDeviceStageView> stage_views;
  stage_views.reserve(serialized_pooling_stages_d_.size());
  for (auto & stage : serialized_pooling_stages_d_) {
    stage_views.push_back(SerializedPoolingDeviceStageView{
      stage.indices.get(), stage.indptr.get(), stage.head_indices.get(), stage.cluster.get(),
      stage.grid_coord.get(), stage.serialized_code.get(), stage.serialized_order.get(),
      stage.serialized_inverse.get()});
  }

  pre_ptr_->generateSerializedPoolingMetadata(
    grid_coord_d_.get(), serialized_code_d_.get(), static_cast<std::uint32_t>(num_voxels_),
    stage_views, serialized_pooling_num_voxels_d_.get());
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    serialized_pooling_num_voxels_.data(), serialized_pooling_num_voxels_d_.get(),
    serialized_pooling_num_voxels_.size() * sizeof(std::uint32_t), cudaMemcpyDeviceToHost,
    stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

bool PTv3TRT::setSerializedPoolingInputShapes()
{
  bool success = true;
  const auto num_orders = static_cast<std::int64_t>(config_.serialization_orders_.size());

  // serialized_pooling_num_voxels_[s] is the input count of stage s (entry 0 is num_voxels_).
  // [s + 1] is its pooled output count, which sets the SegmentCSR output shape and the shape of
  // the metadata consumed by later PTv3 blocks.
  for (std::size_t stage = 0; stage < serialized_pooling_stages_d_.size(); ++stage) {
    const auto prefix = "serialized_pooling_" + std::to_string(stage) + "_";
    const auto in_count = serialized_pooling_num_voxels_[stage];
    const auto out_count = serialized_pooling_num_voxels_[stage + 1];
    success &=
      backbone_trt_ptr_->setInputShape((prefix + "indices").c_str(), nvinfer1::Dims{1, {in_count}});
    success &=
      backbone_trt_ptr_->setInputShape((prefix + "cluster").c_str(), nvinfer1::Dims{1, {in_count}});
    success &= backbone_trt_ptr_->setInputShape(
      (prefix + "indptr").c_str(), nvinfer1::Dims{1, {out_count + 1}});
    success &= backbone_trt_ptr_->setInputShape(
      (prefix + "head_indices").c_str(), nvinfer1::Dims{1, {out_count}});
    success &= backbone_trt_ptr_->setInputShape(
      (prefix + "grid_coord").c_str(), nvinfer1::Dims{2, {out_count, 3}});
    success &= backbone_trt_ptr_->setInputShape(
      (prefix + "serialized_order").c_str(), nvinfer1::Dims{2, {num_orders, out_count}});
    success &= backbone_trt_ptr_->setInputShape(
      (prefix + "serialized_inverse").c_str(), nvinfer1::Dims{2, {num_orders, out_count}});
  }

  return success;
}

CloudFormat PTv3TRT::detectCloudFormat(const cuda_blackboard::CudaPointCloud2 & cloud) const
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

bool PTv3TRT::segment(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
  bool should_publish_filtered_pointcloud, std::unordered_map<std::string, double> & proc_timing)
{
  stop_watch_ptr_->toc("processing/inner", true);
  if (!preProcess(msg_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Pre-process failed. Skipping detection.");
    return false;
  }

  proc_timing.emplace(
    "debug/processing_time/preprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (!inference()) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Inference failed. Skipping detection.");
    return false;
  }

  proc_timing.emplace(
    "debug/processing_time/inference_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (!postProcess(
        msg_ptr->header, should_publish_segmented_pointcloud,
        should_publish_visualization_pointcloud, should_publish_filtered_pointcloud)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Post-process failed. Skipping detection");
    return false;
  }
  proc_timing.emplace(
    "debug/processing_time/postprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  return true;
}

bool PTv3TRT::preProcess(const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  using autoware::cuda_utils::clear_async;

  std::call_once(init_cloud_, [this, &msg_ptr]() {
    input_format_ = detectCloudFormat(*msg_ptr);
    if (input_format_ == CloudFormat::UNKNOWN) {
      throw std::runtime_error(
        "Unsupported point cloud type. Expected one of: XYZIRCAEDT (10 fields), "
        "XYZIRADRT (9 fields), XYZIRC (6 fields), or XYZI (4 fields).");
    }

    const auto requested_output_format = parse_cloud_format_string(config_.filter_output_format_);
    filtered_output_format_ =
      config_.filter_output_format_.empty() ? input_format_ : requested_output_format;
    if (
      filtered_output_format_ == CloudFormat::UNKNOWN ||
      !can_convert_format(input_format_, filtered_output_format_)) {
      throw std::runtime_error(
        "filter.output_format='" + config_.filter_output_format_ +
        "' is not compatible with input format '" + std::string(to_string(input_format_)) + "'.");
    }

    auto append_field = [this](const std::string & name, int offset, int datatype, int count) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      field.count = count;
      filtered_pointcloud_fields_.push_back(field);
    };
    filtered_pointcloud_fields_.clear();
    switch (filtered_output_format_) {
      case CloudFormat::XYZIRCAEDT:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("return_type", 13, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("channel", 14, sensor_msgs::msg::PointField::UINT16, 1);
        append_field("azimuth", 16, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("elevation", 20, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("distance", 24, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("time_stamp", 28, sensor_msgs::msg::PointField::UINT32, 1);
        break;
      case CloudFormat::XYZIRADRT:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("ring", 16, sensor_msgs::msg::PointField::UINT16, 1);
        append_field("azimuth", 18, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("distance", 22, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("return_type", 26, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("time_stamp", 27, sensor_msgs::msg::PointField::FLOAT64, 1);
        break;
      case CloudFormat::XYZIRC:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("return_type", 13, sensor_msgs::msg::PointField::UINT8, 1);
        append_field("channel", 14, sensor_msgs::msg::PointField::UINT16, 1);
        break;
      case CloudFormat::XYZI:
        append_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1);
        append_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1);
        break;
      default:
        throw std::runtime_error("Unsupported filtered point cloud format.");
    }

    RCLCPP_INFO(
      rclcpp::get_logger("ptv3"),
      "Detected input format with %zu fields and point step %zu bytes; filtered output format '%s'",
      get_num_fields(input_format_), get_point_step(input_format_),
      to_string(filtered_output_format_));
  });
  allocateMessages();

  const auto num_points = msg_ptr->height * msg_ptr->width;
  if (config_.source_reconstruction_ == SourceReconstruction::FULL) {
    num_source_points_ = static_cast<std::int64_t>(num_points);
    current_input_data_ = msg_ptr->data.get();
  }

  if (num_points == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Empty pointcloud. Skipping segmentation.");
    return false;
  }

  clear_async(feat_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 4, stream_);
  clear_async(grid_coord_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 3, stream_);
  clear_async(
    serialized_code_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 2, stream_);
  clear_async(pred_labels_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_), stream_);
  clear_async(
    pred_probs_d_.get(),
    static_cast<std::size_t>(config_.max_num_voxels_) * config_.class_names_.size(), stream_);
  clear_async(
    compact_points_d_.get(),
    static_cast<std::size_t>(config_.max_num_voxels_) * sizeof(CloudPointTypeXYZIRCAEDT), stream_);
  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    clear_async(
      cropped_source_points_d_.get(),
      static_cast<std::size_t>(config_.cloud_capacity_) * sizeof(CloudPointTypeXYZIRCAEDT),
      stream_);
  }
  if (config_.source_reconstruction_ != SourceReconstruction::NONE) {
    clear_async(
      reconstructed_features_d_.get(),
      static_cast<std::size_t>(config_.cloud_capacity_) * config_.num_point_feature_size_, stream_);
    clear_async(inverse_map_d_.get(), static_cast<std::size_t>(config_.cloud_capacity_), stream_);
    clear_async(
      reconstructed_labels_d_.get(), static_cast<std::size_t>(config_.cloud_capacity_), stream_);
    clear_async(
      reconstructed_probs_d_.get(),
      static_cast<std::size_t>(config_.cloud_capacity_) * config_.class_names_.size(), stream_);
  }

  std::size_t num_cropped_points = 0;
  num_voxels_ = pre_ptr_->generateFeatures(
    msg_ptr->data.get(), input_format_, num_points, feat_d_.get(), grid_coord_d_.get(),
    serialized_code_d_.get(), compact_points_d_.get(),
    config_.source_reconstruction_ != SourceReconstruction::NONE ? reconstructed_features_d_.get()
                                                                 : nullptr,
    config_.source_reconstruction_ == SourceReconstruction::PARTIAL ? cropped_source_points_d_.get()
                                                                    : nullptr,
    config_.source_reconstruction_ != SourceReconstruction::NONE ? inverse_map_d_.get() : nullptr,
    &num_cropped_points);
  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    num_cropped_points_ = static_cast<std::int64_t>(num_cropped_points);
  }

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

  precomputeSerializedPoolingMetadata();

  backbone_trt_ptr_->setInputShape("grid_coord", nvinfer1::Dims{2, {num_voxels_, 3}});
  backbone_trt_ptr_->setInputShape("feat", nvinfer1::Dims{2, {num_voxels_, 4}});
  backbone_trt_ptr_->setInputShape("serialized_code", nvinfer1::Dims{2, {2, num_voxels_}});

  if (!setSerializedPoolingInputShapes()) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Failed to set serialized pooling input shapes.");
    return false;
  }

  return true;
}

bool PTv3TRT::inference()
{
  if (!backbone_trt_ptr_->enqueueV3(stream_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Fail to enqueue backbone and skip to detect.");
    return false;
  }

  if (seg3d_head_trt_ptr_) {
    seg3d_head_trt_ptr_->setInputShape(
      "point_feat", nvinfer1::Dims{2, {num_voxels_, config_.backbone_feat_dim_}});
    if (!seg3d_head_trt_ptr_->enqueueV3(stream_)) {
      RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Fail to enqueue seg3d head and skip to detect.");
      return false;
    }
  }

  return true;
}

bool PTv3TRT::postProcess(
  const std_msgs::msg::Header & header, bool should_publish_segmented_pointcloud,
  bool should_publish_visualization_pointcloud, bool should_publish_filtered_pointcloud)
{
  // Segmentation pointcloud
  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    post_ptr_->reconstructPartial(
      inverse_map_d_.get(), pred_labels_d_.get(), pred_probs_d_.get(),
      reconstructed_labels_d_.get(), reconstructed_probs_d_.get(), config_.class_names_.size(),
      num_cropped_points_, num_voxels_);
  }
  if (config_.source_reconstruction_ == SourceReconstruction::FULL) {
    post_ptr_->reconstructFull(
      pre_ptr_->cropMask(), pre_ptr_->cropIndices(), inverse_map_d_.get(), pred_labels_d_.get(),
      pred_probs_d_.get(), reconstructed_labels_d_.get(), reconstructed_probs_d_.get(),
      config_.class_names_.size(), num_source_points_, num_voxels_);
  }

  const auto source_features = config_.source_reconstruction_ != SourceReconstruction::NONE
                                 ? reconstructed_features_d_.get()
                                 : feat_d_.get();
  const auto source_labels = config_.source_reconstruction_ != SourceReconstruction::NONE
                               ? reconstructed_labels_d_.get()
                               : pred_labels_d_.get();
  const auto source_probs = config_.source_reconstruction_ != SourceReconstruction::NONE
                              ? reconstructed_probs_d_.get()
                              : pred_probs_d_.get();
  const auto source_points = config_.source_reconstruction_ == SourceReconstruction::FULL
                               ? current_input_data_
                             : config_.source_reconstruction_ == SourceReconstruction::PARTIAL
                               ? cropped_source_points_d_.get()
                               : compact_points_d_.get();
  const auto num_source_output_points =
    config_.source_reconstruction_ == SourceReconstruction::FULL      ? num_source_points_
    : config_.source_reconstruction_ == SourceReconstruction::PARTIAL ? num_cropped_points_
                                                                      : num_voxels_;

  if (should_publish_segmented_pointcloud) {
    post_ptr_->createSegmentationPointcloud(
      source_features, source_labels, source_probs, segmented_points_msg_ptr_->data.get(),
      config_.class_names_.size(), num_source_output_points);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    segmented_points_msg_ptr_->header = header;
    segmented_points_msg_ptr_->width = static_cast<std::uint32_t>(num_source_output_points);
    publish_segmented_pointcloud_(std::move(segmented_points_msg_ptr_));
    segmented_points_msg_ptr_ = nullptr;
  }

  // Visualization pointcloud
  if (should_publish_visualization_pointcloud) {
    post_ptr_->createVisualizationPointcloud(
      source_features, source_labels,
      reinterpret_cast<float *>(visualization_points_msg_ptr_->data.get()),
      config_.class_names_.size(), num_source_output_points);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    visualization_points_msg_ptr_->header = header;
    visualization_points_msg_ptr_->width = static_cast<std::uint32_t>(num_source_output_points);
    publish_visualization_pointcloud_(std::move(visualization_points_msg_ptr_));
    visualization_points_msg_ptr_ = nullptr;
  }

  if (should_publish_filtered_pointcloud) {
    const auto num_filtered_points = post_ptr_->createFilteredPointcloud(
      source_points, input_format_, filtered_output_format_, source_probs,
      filtered_points_msg_ptr_->data.get(), config_.class_names_.size(), num_source_output_points);
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    filtered_points_msg_ptr_->header = header;
    filtered_points_msg_ptr_->width = num_filtered_points;
    publish_filtered_pointcloud_(std::move(filtered_points_msg_ptr_));
    filtered_points_msg_ptr_ = nullptr;
  }

  allocateMessages();
  return true;
}

}  //  namespace autoware::ptv3
