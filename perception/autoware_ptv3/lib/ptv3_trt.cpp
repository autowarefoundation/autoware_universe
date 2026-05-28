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

#include "autoware/ptv3/preprocess/backbone_preprocess.hpp"
#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_utils.hpp>
#include <autoware/point_types/memory.hpp>
#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cstdint>
#include <exception>
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
constexpr const char * k_logger = "ptv3";

const char * to_trt_dtype_name(const nvinfer1::DataType dtype)
{
  switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
      return "FLOAT";
    case nvinfer1::DataType::kHALF:
      return "HALF";
    case nvinfer1::DataType::kINT8:
      return "INT8";
    case nvinfer1::DataType::kINT32:
      return "INT32";
    case nvinfer1::DataType::kBOOL:
      return "BOOL";
    case nvinfer1::DataType::kINT64:
      return "INT64";
    default:
      return "UNKNOWN";
  }
}

void require_same_dtype(
  const nvinfer1::DataType expected, const nvinfer1::DataType actual, const char * tensor_name)
{
  if (actual != expected) {
    throw std::runtime_error(
      std::string("Mixed TensorRT float output dtypes are not supported. Tensor '") + tensor_name +
      "' has " + to_trt_dtype_name(actual) + ", expected " + to_trt_dtype_name(expected) + ".");
  }
}
}  // namespace

PTv3TRT::PTv3TRT(
  const tensorrt_common::TrtCommonConfig & backbone_trt_config,
  const std::optional<tensorrt_common::TrtCommonConfig> & seg3d_head_trt_config,
  const std::optional<tensorrt_common::TrtCommonConfig> & det3d_head_trt_config,
  const PTv3Config & config)
: config_(config)
{
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing/inner");

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
  create_point_fields();
  init_ptr();
  init_backbone_trt(backbone_trt_config);
  if (config_.use_seg3d_head_) {
    if (!seg3d_head_trt_config.has_value()) {
      throw std::runtime_error("seg3d_head_trt_config is required when segmentation3d.use_head.");
    }
    init_seg3d_head_trt(*seg3d_head_trt_config);
  }
  if (config_.use_det3d_head_) {
    if (!det3d_head_trt_config.has_value()) {
      throw std::runtime_error("det3d_head_trt_config is required when detection3d.use_head.");
    }
    init_det3d_head_trt(*det3d_head_trt_config);
  }
}

PTv3TRT::~PTv3TRT()
{
  if (stream_) {
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    CHECK_CUDA_ERROR(cudaStreamDestroy(stream_));
  }
}

void PTv3TRT::set_publish_segmented_pointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_segmented_pointcloud_ = std::move(func);
}

void PTv3TRT::set_publish_visualization_pointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_visualization_pointcloud_ = std::move(func);
}

void PTv3TRT::set_publish_filtered_pointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  publish_filtered_pointcloud_ = std::move(func);
}

nvinfer1::DataType PTv3TRT::bind_float_output(
  autoware::tensorrt_common::TrtCommon * trt_ptr, const char * tensor_name, float * fp32_buffer,
  CudaUniquePtr<__half[]> & fp16_buffer, std::size_t num_elements)
{
  const auto dtype = trt_ptr->getTensorDataType(tensor_name);
  if (!dtype.has_value()) {
    throw std::runtime_error(
      std::string("Failed to query TensorRT dtype for ") + tensor_name + ".");
  }
  if (*dtype == nvinfer1::DataType::kFLOAT) {
    trt_ptr->setTensorAddress(tensor_name, fp32_buffer);
    return *dtype;
  }
  if (*dtype == nvinfer1::DataType::kHALF) {
    fp16_buffer = autoware::cuda_utils::make_unique<__half[]>(num_elements);
    trt_ptr->setTensorAddress(tensor_name, fp16_buffer.get());
    return *dtype;
  }
  throw std::runtime_error(
    std::string("Unsupported TensorRT dtype for float output '") + tensor_name + "'.");
}

void PTv3TRT::allocate_messages()
{
  if (!config_.use_seg3d_head_) {
    return;
  }

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

void PTv3TRT::init_ptr()
{
  // Backbone input buffers
  grid_coord_d_ = autoware::cuda_utils::make_unique<std::int32_t[]>(config_.max_num_voxels_ * 3);
  feat_d_ = autoware::cuda_utils::make_unique<float[]>(config_.max_num_voxels_ * 4);
  serialized_code_d_ =
    autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_ * 2);

  // Backbone output / head input buffers
  bb_point_feat_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.max_num_voxels_ * config_.backbone_feat_dim_);
  bb_point_grid_coord_d_ =
    autoware::cuda_utils::make_unique<std::int32_t[]>(config_.max_num_voxels_ * 3);
  bb_point_offset_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(1);

  // Point cloud buffers
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

  pre_ptr_ = std::make_unique<BackbonePreprocess>(config_, stream_);

  // Seg head output buffers
  if (config_.use_seg3d_head_) {
    pred_labels_d_ = autoware::cuda_utils::make_unique<std::int64_t[]>(config_.max_num_voxels_);
    pred_probs_d_ = autoware::cuda_utils::make_unique<float[]>(
      config_.max_num_voxels_ * config_.class_names_.size());
    post_ptr_ = std::make_unique<Seg3dPostprocess>(config_, stream_);
  }

  // Det head output buffers
  if (config_.use_det3d_head_) {
    const auto det_grid_size = config_.det_grid_x_size_ * config_.det_grid_y_size_;
    const auto det_class_size = config_.detection_class_names_.size();

    if (config_.detection_head_type_ == DetectionHeadType::CenterHead) {
      heatmap_d_ = autoware::cuda_utils::make_unique<float[]>(det_grid_size * det_class_size);
      reg_d_ =
        autoware::cuda_utils::make_unique<float[]>(det_grid_size * config_.head_out_reg_size_);
      height_d_ =
        autoware::cuda_utils::make_unique<float[]>(det_grid_size * config_.head_out_height_size_);
      dim_d_ =
        autoware::cuda_utils::make_unique<float[]>(det_grid_size * config_.head_out_dim_size_);
      rot_d_ =
        autoware::cuda_utils::make_unique<float[]>(det_grid_size * config_.head_out_rot_size_);
      if (config_.has_twist_) {
        vel_d_ =
          autoware::cuda_utils::make_unique<float[]>(det_grid_size * config_.head_out_vel_size_);
      }
      centerhead_post_ptr_ = std::make_unique<Det3dCenterHeadPostprocess>(config_, stream_);
    } else {
      dense_heatmap_d_ = autoware::cuda_utils::make_unique<float[]>(det_grid_size * det_class_size);
      query_heatmap_score_d_ =
        autoware::cuda_utils::make_unique<float[]>(det_class_size * config_.num_proposals_);
      query_labels_i32_d_ =
        autoware::cuda_utils::make_unique<std::int32_t[]>(config_.num_proposals_);
      query_labels_i64_d_ =
        autoware::cuda_utils::make_unique<std::int64_t[]>(config_.num_proposals_);
      heatmap_d_ =
        autoware::cuda_utils::make_unique<float[]>(det_class_size * config_.num_proposals_);
      center_d_ = autoware::cuda_utils::make_unique<float[]>(2 * config_.num_proposals_);
      height_d_ = autoware::cuda_utils::make_unique<float[]>(config_.num_proposals_);
      dim_d_ = autoware::cuda_utils::make_unique<float[]>(3 * config_.num_proposals_);
      rot_d_ = autoware::cuda_utils::make_unique<float[]>(2 * config_.num_proposals_);
      if (config_.has_twist_) {
        vel_d_ = autoware::cuda_utils::make_unique<float[]>(2 * config_.num_proposals_);
      }
      transhead_post_ptr_ = std::make_unique<Det3dTransHeadPostprocess>(config_, stream_);
    }
  }

  allocate_messages();
}

void PTv3TRT::create_point_fields()
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

void PTv3TRT::init_backbone_trt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  // Inputs
  network_io.emplace_back("grid_coord", nvinfer1::Dims{2, {-1, 3}});
  network_io.emplace_back("feat", nvinfer1::Dims{2, {-1, 4}});
  network_io.emplace_back("serialized_code", nvinfer1::Dims{2, {2, -1}});

  // Outputs: point_feat [N,64], point_grid_coord [N,3], point_offset [1]
  network_io.emplace_back("point_feat", nvinfer1::Dims{2, {-1, config_.backbone_feat_dim_}});
  network_io.emplace_back("point_grid_coord", nvinfer1::Dims{2, {-1, 3}});
  network_io.emplace_back("point_offset", nvinfer1::Dims{1, {1}});

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
}

void PTv3TRT::init_seg3d_head_trt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  network_io.emplace_back("point_feat", nvinfer1::Dims{2, {-1, config_.backbone_feat_dim_}});
  network_io.emplace_back("pred_labels", nvinfer1::Dims{1, {-1}});
  network_io.emplace_back(
    "pred_probs", nvinfer1::Dims{2, {-1, static_cast<std::int64_t>(config_.class_names_.size())}});

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

  pred_probs_dtype_ = bind_float_output(
    seg3d_head_trt_ptr_.get(), "pred_probs", pred_probs_d_.get(), pred_probs_fp16_d_,
    static_cast<std::size_t>(config_.max_num_voxels_) * config_.class_names_.size());

  if (
    pred_probs_dtype_ == nvinfer1::DataType::kHALF &&
    config_.source_reconstruction_ != SourceReconstruction::NONE) {
    reconstructed_probs_fp16_d_ = autoware::cuda_utils::make_unique<__half[]>(
      config_.cloud_capacity_ * config_.class_names_.size());
  }
}

void PTv3TRT::init_det3d_head_trt(const tensorrt_common::TrtCommonConfig & trt_config)
{
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  // Inputs (dynamic on num_points)
  // Note: point_offset is folded out of the det3d head ONNX by the exporter (batch_size=1),
  // so it is not registered here.
  network_io.emplace_back("point_feat", nvinfer1::Dims{2, {-1, config_.backbone_feat_dim_}});
  network_io.emplace_back("point_grid_coord", nvinfer1::Dims{2, {-1, 3}});

  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims;
  profile_dims.emplace_back(
    "point_feat", nvinfer1::Dims{2, {config_.voxels_num_[0], config_.backbone_feat_dim_}},
    nvinfer1::Dims{2, {config_.voxels_num_[1], config_.backbone_feat_dim_}},
    nvinfer1::Dims{2, {config_.voxels_num_[2], config_.backbone_feat_dim_}});
  profile_dims.emplace_back(
    "point_grid_coord", nvinfer1::Dims{2, {config_.voxels_num_[0], 3}},
    nvinfer1::Dims{2, {config_.voxels_num_[1], 3}}, nvinfer1::Dims{2, {config_.voxels_num_[2], 3}});

  if (config_.detection_head_type_ == DetectionHeadType::CenterHead) {
    const auto det_grid_size = config_.det_grid_x_size_ * config_.det_grid_y_size_;
    const auto det_cls = static_cast<std::int64_t>(config_.detection_class_names_.size());
    const auto gx = static_cast<std::int64_t>(config_.det_grid_x_size_);
    const auto gy = static_cast<std::int64_t>(config_.det_grid_y_size_);

    network_io.emplace_back("heatmap", nvinfer1::Dims{4, {1, det_cls, gy, gx}});
    network_io.emplace_back(
      "reg", nvinfer1::Dims{4, {1, static_cast<std::int64_t>(config_.head_out_reg_size_), gy, gx}});
    network_io.emplace_back(
      "height",
      nvinfer1::Dims{4, {1, static_cast<std::int64_t>(config_.head_out_height_size_), gy, gx}});
    network_io.emplace_back(
      "dim", nvinfer1::Dims{4, {1, static_cast<std::int64_t>(config_.head_out_dim_size_), gy, gx}});
    network_io.emplace_back(
      "rot", nvinfer1::Dims{4, {1, static_cast<std::int64_t>(config_.head_out_rot_size_), gy, gx}});
    if (config_.has_twist_) {
      network_io.emplace_back(
        "vel",
        nvinfer1::Dims{4, {1, static_cast<std::int64_t>(config_.head_out_vel_size_), gy, gx}});
    }

    det3d_head_trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
      trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
      std::vector<std::string>{config_.plugins_path_});

    if (!det3d_head_trt_ptr_->setup(
          std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims),
          std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io))) {
      throw std::runtime_error("Failed to setup det3d_head (CenterHead) TRT engine.");
    }

    det3d_head_trt_ptr_->setTensorAddress("point_feat", bb_point_feat_d_.get());
    det3d_head_trt_ptr_->setTensorAddress("point_grid_coord", bb_point_grid_coord_d_.get());

    heatmap_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "heatmap", heatmap_d_.get(), heatmap_fp16_d_,
      det_grid_size * config_.detection_class_names_.size());
    reg_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "reg", reg_d_.get(), reg_fp16_d_,
      det_grid_size * config_.head_out_reg_size_);
    height_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "height", height_d_.get(), height_fp16_d_,
      det_grid_size * config_.head_out_height_size_);
    dim_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "dim", dim_d_.get(), dim_fp16_d_,
      det_grid_size * config_.head_out_dim_size_);
    rot_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "rot", rot_d_.get(), rot_fp16_d_,
      det_grid_size * config_.head_out_rot_size_);
    require_same_dtype(heatmap_dtype_, reg_dtype_, "reg");
    require_same_dtype(heatmap_dtype_, height_dtype_, "height");
    require_same_dtype(heatmap_dtype_, dim_dtype_, "dim");
    require_same_dtype(heatmap_dtype_, rot_dtype_, "rot");
    if (config_.has_twist_) {
      vel_dtype_ = bind_float_output(
        det3d_head_trt_ptr_.get(), "vel", vel_d_.get(), vel_fp16_d_,
        det_grid_size * config_.head_out_vel_size_);
      require_same_dtype(heatmap_dtype_, vel_dtype_, "vel");
    }
  } else {
    // TransHead
    const auto det_grid_size = config_.det_grid_x_size_ * config_.det_grid_y_size_;
    const auto det_cls = static_cast<std::int64_t>(config_.detection_class_names_.size());
    const auto gx = static_cast<std::int64_t>(config_.det_grid_x_size_);
    const auto gy = static_cast<std::int64_t>(config_.det_grid_y_size_);
    const auto np = static_cast<std::int64_t>(config_.num_proposals_);

    network_io.emplace_back("dense_heatmap", nvinfer1::Dims{4, {1, det_cls, gy, gx}});
    network_io.emplace_back("query_heatmap_score", nvinfer1::Dims{3, {1, det_cls, np}});
    network_io.emplace_back("query_labels", nvinfer1::Dims{2, {1, np}});
    network_io.emplace_back("heatmap", nvinfer1::Dims{3, {1, det_cls, np}});
    network_io.emplace_back("center", nvinfer1::Dims{3, {1, 2, np}});
    network_io.emplace_back("height", nvinfer1::Dims{3, {1, 1, np}});
    network_io.emplace_back("dim", nvinfer1::Dims{3, {1, 3, np}});
    network_io.emplace_back("rot", nvinfer1::Dims{3, {1, 2, np}});
    if (config_.has_twist_) {
      network_io.emplace_back("vel", nvinfer1::Dims{3, {1, 2, np}});
    }

    det3d_head_trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(
      trt_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
      std::vector<std::string>{config_.plugins_path_});

    if (!det3d_head_trt_ptr_->setup(
          std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims),
          std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io))) {
      throw std::runtime_error("Failed to setup det3d_head (TransHead) TRT engine.");
    }

    det3d_head_trt_ptr_->setTensorAddress("point_feat", bb_point_feat_d_.get());
    det3d_head_trt_ptr_->setTensorAddress("point_grid_coord", bb_point_grid_coord_d_.get());

    const auto query_labels_dtype = det3d_head_trt_ptr_->getTensorDataType("query_labels");
    if (!query_labels_dtype.has_value()) {
      throw std::runtime_error("Failed to query TensorRT dtype for query_labels.");
    }
    query_labels_dtype_ = *query_labels_dtype;
    if (query_labels_dtype_ == nvinfer1::DataType::kINT32) {
      det3d_head_trt_ptr_->setTensorAddress("query_labels", query_labels_i32_d_.get());
    } else if (query_labels_dtype_ == nvinfer1::DataType::kINT64) {
      det3d_head_trt_ptr_->setTensorAddress("query_labels", query_labels_i64_d_.get());
    } else {
      throw std::runtime_error(
        std::string("Unsupported TensorRT dtype for query_labels: ") +
        to_trt_dtype_name(query_labels_dtype_) + ". Expected INT32 or INT64.");
    }

    dense_heatmap_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "dense_heatmap", dense_heatmap_d_.get(), dense_heatmap_fp16_d_,
      det_grid_size * config_.detection_class_names_.size());
    query_heatmap_score_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "query_heatmap_score", query_heatmap_score_d_.get(),
      query_heatmap_score_fp16_d_, config_.detection_class_names_.size() * config_.num_proposals_);
    heatmap_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "heatmap", heatmap_d_.get(), heatmap_fp16_d_,
      config_.detection_class_names_.size() * config_.num_proposals_);
    center_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "center", center_d_.get(), center_fp16_d_,
      2 * config_.num_proposals_);
    height_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "height", height_d_.get(), height_fp16_d_, config_.num_proposals_);
    dim_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "dim", dim_d_.get(), dim_fp16_d_, 3 * config_.num_proposals_);
    rot_dtype_ = bind_float_output(
      det3d_head_trt_ptr_.get(), "rot", rot_d_.get(), rot_fp16_d_, 2 * config_.num_proposals_);
    require_same_dtype(heatmap_dtype_, dense_heatmap_dtype_, "dense_heatmap");
    require_same_dtype(heatmap_dtype_, query_heatmap_score_dtype_, "query_heatmap_score");
    require_same_dtype(heatmap_dtype_, center_dtype_, "center");
    require_same_dtype(heatmap_dtype_, height_dtype_, "height");
    require_same_dtype(heatmap_dtype_, dim_dtype_, "dim");
    require_same_dtype(heatmap_dtype_, rot_dtype_, "rot");
    if (config_.has_twist_) {
      vel_dtype_ = bind_float_output(
        det3d_head_trt_ptr_.get(), "vel", vel_d_.get(), vel_fp16_d_, 2 * config_.num_proposals_);
      require_same_dtype(heatmap_dtype_, vel_dtype_, "vel");
    }
  }
}

CloudFormat PTv3TRT::detect_cloud_format(const cuda_blackboard::CudaPointCloud2 & cloud)
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
  std::optional<std::vector<Box3D>> dummy_boxes;
  return infer(
    msg_ptr, should_publish_segmented_pointcloud, should_publish_visualization_pointcloud,
    should_publish_filtered_pointcloud, false, dummy_boxes, proc_timing);
}

bool PTv3TRT::detect(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  std::vector<Box3D> & det_boxes3d, std::unordered_map<std::string, double> & proc_timing)
{
  std::optional<std::vector<Box3D>> detected_boxes;
  const bool success = infer(msg_ptr, false, false, false, true, detected_boxes, proc_timing);
  det_boxes3d = detected_boxes.value_or(std::vector<Box3D>{});
  return success && detected_boxes.has_value();
}

bool PTv3TRT::infer(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
  bool should_publish_filtered_pointcloud, bool should_detect_objects,
  std::optional<std::vector<Box3D>> & det_boxes3d,
  std::unordered_map<std::string, double> & proc_timing)
{
  det_boxes3d.reset();

  stop_watch_ptr_->toc("processing/inner", true);
  if (!pre_process(msg_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Pre-process failed.");
    return false;
  }
  proc_timing.emplace(
    "debug/processing_time/preprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (!infer_backbone()) {
    RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Backbone inference failed.");
    return false;
  }

  const bool should_run_seg3d =
    config_.use_seg3d_head_ &&
    (should_publish_segmented_pointcloud || should_publish_visualization_pointcloud ||
     should_publish_filtered_pointcloud);
  const bool should_run_det3d = config_.use_det3d_head_ && should_detect_objects;

  bool seg_ok = !should_run_seg3d;
  bool det_ok = !should_run_det3d;
  bool seg_post_ok = !should_run_seg3d;
  bool det_post_ok = !should_run_det3d;

  if (should_run_seg3d) {
    seg_ok = infer_seg3d_head();
    if (!seg_ok) {
      RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Seg head inference failed.");
    }
  }

  if (should_run_det3d) {
    det_ok = infer_det3d_head();
    if (!det_ok) {
      RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Det head inference failed.");
    }
  }

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  proc_timing.emplace(
    "debug/processing_time/inference_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (seg_ok && should_run_seg3d) {
    try {
      if (!post_process_seg3d(
            msg_ptr->header, should_publish_segmented_pointcloud,
            should_publish_visualization_pointcloud, should_publish_filtered_pointcloud)) {
        RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Seg post-process failed.");
        seg_post_ok = false;
      } else {
        seg_post_ok = true;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Seg post-process failed: %s", e.what());
      seg_post_ok = false;
    }
  }

  if (det_ok && should_run_det3d) {
    std::vector<Box3D> detected_boxes;
    try {
      if (post_process_det3d(detected_boxes)) {
        det_boxes3d = std::move(detected_boxes);
        det_post_ok = true;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Det post-process failed.");
        det_post_ok = false;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Det post-process failed: %s", e.what());
      det_post_ok = false;
    }
  }

  proc_timing.emplace(
    "debug/processing_time/postprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  return (should_run_seg3d && seg_ok && seg_post_ok) || (should_run_det3d && det_ok && det_post_ok);
}

bool PTv3TRT::pre_process(const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  using autoware::cuda_utils::clear_async;

  std::call_once(init_cloud_, [this, &msg_ptr]() {
    input_format_ = detect_cloud_format(*msg_ptr);
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
      rclcpp::get_logger(k_logger),
      "Detected input format with %zu fields and point step %zu bytes; filtered output format '%s'",
      get_num_fields(input_format_), get_point_step(input_format_),
      to_string(filtered_output_format_));
  });
  allocate_messages();

  const auto raw_num_points =
    static_cast<std::size_t>(msg_ptr->height) * static_cast<std::size_t>(msg_ptr->width);
  const auto num_points =
    std::min<std::size_t>(raw_num_points, static_cast<std::size_t>(config_.cloud_capacity_));
  if (raw_num_points > static_cast<std::size_t>(config_.cloud_capacity_)) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(k_logger), "Pointcloud size ("
                                      << raw_num_points << ") exceeds cloud_capacity ("
                                      << config_.cloud_capacity_ << "). Clipping.");
  }
  if (config_.source_reconstruction_ == SourceReconstruction::FULL) {
    num_source_points_ = static_cast<std::int64_t>(num_points);
    current_input_data_ = msg_ptr->data.get();
  }

  if (num_points == 0) {
    RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Empty pointcloud. Skipping.");
    return false;
  }

  clear_async(feat_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 4, stream_);
  clear_async(grid_coord_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 3, stream_);
  clear_async(
    serialized_code_d_.get(), static_cast<std::size_t>(config_.max_num_voxels_) * 2, stream_);
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
    if (reconstructed_probs_fp16_d_) {
      CHECK_CUDA_ERROR(cudaMemsetAsync(
        reconstructed_probs_fp16_d_.get(), 0,
        static_cast<std::size_t>(config_.cloud_capacity_) * config_.class_names_.size() *
          sizeof(__half),
        stream_));
    }
  }

  std::size_t num_cropped_points = 0;
  std::size_t unclipped_num_voxels = 0;
  num_voxels_ = pre_ptr_->generate_features(
    msg_ptr->data.get(), input_format_, static_cast<unsigned int>(num_points), feat_d_.get(),
    grid_coord_d_.get(), serialized_code_d_.get(), compact_points_d_.get(),
    config_.source_reconstruction_ != SourceReconstruction::NONE ? reconstructed_features_d_.get()
                                                                 : nullptr,
    config_.source_reconstruction_ == SourceReconstruction::PARTIAL ? cropped_source_points_d_.get()
                                                                    : nullptr,
    config_.source_reconstruction_ != SourceReconstruction::NONE ? inverse_map_d_.get() : nullptr,
    &num_cropped_points, &unclipped_num_voxels);
  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    num_cropped_points_ = static_cast<std::int64_t>(num_cropped_points);
  }
  if (unclipped_num_voxels > static_cast<std::size_t>(config_.max_num_voxels_)) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(k_logger), "Voxel count (" << unclipped_num_voxels
                                                    << ") exceeds max_num_voxels ("
                                                    << config_.max_num_voxels_ << "). Clipping.");
  }

  if (num_voxels_ < config_.min_num_voxels_) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(k_logger), "Too few voxels (" << num_voxels_
                                                       << ") for the optimization profile ("
                                                       << config_.min_num_voxels_ << ")");
    return false;
  }
  backbone_trt_ptr_->setInputShape("grid_coord", nvinfer1::Dims{2, {num_voxels_, 3}});
  backbone_trt_ptr_->setInputShape("feat", nvinfer1::Dims{2, {num_voxels_, 4}});
  backbone_trt_ptr_->setInputShape("serialized_code", nvinfer1::Dims{2, {2, num_voxels_}});

  return true;
}

bool PTv3TRT::infer_backbone()
{
  const auto status = backbone_trt_ptr_->enqueueV3(stream_);
  if (!status) {
    RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Backbone enqueue failed.");
    return false;
  }
  return true;
}

bool PTv3TRT::infer_seg3d_head()
{
  seg3d_head_trt_ptr_->setInputShape(
    "point_feat", nvinfer1::Dims{2, {num_voxels_, config_.backbone_feat_dim_}});
  const auto status = seg3d_head_trt_ptr_->enqueueV3(stream_);
  if (!status) {
    RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Seg head enqueue failed.");
    return false;
  }
  return true;
}

bool PTv3TRT::infer_det3d_head()
{
  det3d_head_trt_ptr_->setInputShape(
    "point_feat", nvinfer1::Dims{2, {num_voxels_, config_.backbone_feat_dim_}});
  det3d_head_trt_ptr_->setInputShape("point_grid_coord", nvinfer1::Dims{2, {num_voxels_, 3}});
  const auto status = det3d_head_trt_ptr_->enqueueV3(stream_);
  if (!status) {
    RCLCPP_ERROR(rclcpp::get_logger(k_logger), "Det head enqueue failed.");
    return false;
  }
  return true;
}

bool PTv3TRT::post_process_seg3d(
  const std_msgs::msg::Header & header, bool should_publish_segmented_pointcloud,
  bool should_publish_visualization_pointcloud, bool should_publish_filtered_pointcloud)
{
  const bool probs_fp16 = (pred_probs_dtype_ == nvinfer1::DataType::kHALF);

  const bool in_reconstruction = (config_.source_reconstruction_ != SourceReconstruction::NONE);

  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    if (probs_fp16) {
      post_ptr_->reconstruct_partial(
        inverse_map_d_.get(), pred_labels_d_.get(), pred_probs_fp16_d_.get(),
        reconstructed_labels_d_.get(), reconstructed_probs_fp16_d_.get(),
        config_.class_names_.size(), num_cropped_points_, num_voxels_);
    } else {
      post_ptr_->reconstruct_partial(
        inverse_map_d_.get(), pred_labels_d_.get(), pred_probs_d_.get(),
        reconstructed_labels_d_.get(), reconstructed_probs_d_.get(), config_.class_names_.size(),
        num_cropped_points_, num_voxels_);
    }
  }
  if (config_.source_reconstruction_ == SourceReconstruction::FULL) {
    if (probs_fp16) {
      post_ptr_->reconstruct_full(
        pre_ptr_->crop_mask(), pre_ptr_->crop_indices(), inverse_map_d_.get(), pred_labels_d_.get(),
        pred_probs_fp16_d_.get(), reconstructed_labels_d_.get(), reconstructed_probs_fp16_d_.get(),
        config_.class_names_.size(), num_source_points_, num_voxels_);
    } else {
      post_ptr_->reconstruct_full(
        pre_ptr_->crop_mask(), pre_ptr_->crop_indices(), inverse_map_d_.get(), pred_labels_d_.get(),
        pred_probs_d_.get(), reconstructed_labels_d_.get(), reconstructed_probs_d_.get(),
        config_.class_names_.size(), num_source_points_, num_voxels_);
    }
  }

  const auto source_features = in_reconstruction ? reconstructed_features_d_.get() : feat_d_.get();
  const auto source_labels =
    in_reconstruction ? reconstructed_labels_d_.get() : pred_labels_d_.get();
  const __half * source_probs_fp16 =
    in_reconstruction ? reconstructed_probs_fp16_d_.get() : pred_probs_fp16_d_.get();
  const float * source_probs_fp32 =
    in_reconstruction ? reconstructed_probs_d_.get() : pred_probs_d_.get();

  const auto source_points = config_.source_reconstruction_ == SourceReconstruction::FULL
                               ? current_input_data_
                             : config_.source_reconstruction_ == SourceReconstruction::PARTIAL
                               ? static_cast<const void *>(cropped_source_points_d_.get())
                               : static_cast<const void *>(compact_points_d_.get());
  const auto num_source_output_points =
    config_.source_reconstruction_ == SourceReconstruction::FULL      ? num_source_points_
    : config_.source_reconstruction_ == SourceReconstruction::PARTIAL ? num_cropped_points_
                                                                      : num_voxels_;

  if (should_publish_segmented_pointcloud) {
    if (probs_fp16) {
      post_ptr_->create_segmentation_pointcloud(
        source_features, source_labels, source_probs_fp16, segmented_points_msg_ptr_->data.get(),
        config_.class_names_.size(), num_source_output_points);
    } else {
      post_ptr_->create_segmentation_pointcloud(
        source_features, source_labels, source_probs_fp32, segmented_points_msg_ptr_->data.get(),
        config_.class_names_.size(), num_source_output_points);
    }
    segmented_points_msg_ptr_->header = header;
    segmented_points_msg_ptr_->width = static_cast<std::uint32_t>(num_source_output_points);
    publish_segmented_pointcloud_(std::move(segmented_points_msg_ptr_));
    segmented_points_msg_ptr_ = nullptr;
  }

  if (should_publish_visualization_pointcloud) {
    post_ptr_->create_visualization_pointcloud(
      source_features, source_labels,
      reinterpret_cast<float *>(visualization_points_msg_ptr_->data.get()),
      config_.class_names_.size(), num_source_output_points);
    visualization_points_msg_ptr_->header = header;
    visualization_points_msg_ptr_->width = static_cast<std::uint32_t>(num_source_output_points);
    publish_visualization_pointcloud_(std::move(visualization_points_msg_ptr_));
    visualization_points_msg_ptr_ = nullptr;
  }

  if (should_publish_filtered_pointcloud) {
    std::size_t num_filtered_points = 0;
    if (probs_fp16) {
      num_filtered_points = post_ptr_->create_filtered_pointcloud(
        source_points, input_format_, filtered_output_format_, source_probs_fp16,
        filtered_points_msg_ptr_->data.get(), config_.class_names_.size(),
        num_source_output_points);
    } else {
      num_filtered_points = post_ptr_->create_filtered_pointcloud(
        source_points, input_format_, filtered_output_format_, source_probs_fp32,
        filtered_points_msg_ptr_->data.get(), config_.class_names_.size(),
        num_source_output_points);
    }
    filtered_points_msg_ptr_->header = header;
    filtered_points_msg_ptr_->width = num_filtered_points;
    publish_filtered_pointcloud_(std::move(filtered_points_msg_ptr_));
    filtered_points_msg_ptr_ = nullptr;
  }

  allocate_messages();
  return true;
}

bool PTv3TRT::post_process_det3d(std::vector<Box3D> & det_boxes3d)
{
  if (config_.detection_head_type_ == DetectionHeadType::CenterHead) {
    if (heatmap_dtype_ == nvinfer1::DataType::kHALF) {
      CHECK_CUDA_ERROR(centerhead_post_ptr_->process(
        heatmap_fp16_d_.get(), reg_fp16_d_.get(), height_fp16_d_.get(), dim_fp16_d_.get(),
        rot_fp16_d_.get(), config_.has_twist_ ? vel_fp16_d_.get() : nullptr, stream_));
    } else {
      CHECK_CUDA_ERROR(centerhead_post_ptr_->process(
        heatmap_d_.get(), reg_d_.get(), height_d_.get(), dim_d_.get(), rot_d_.get(),
        config_.has_twist_ ? vel_d_.get() : nullptr, stream_));
    }
  } else {
    if (heatmap_dtype_ == nvinfer1::DataType::kHALF) {
      if (query_labels_dtype_ == nvinfer1::DataType::kINT32) {
        CHECK_CUDA_ERROR(transhead_post_ptr_->process(
          query_heatmap_score_fp16_d_.get(), query_labels_i32_d_.get(), heatmap_fp16_d_.get(),
          center_fp16_d_.get(), height_fp16_d_.get(), dim_fp16_d_.get(), rot_fp16_d_.get(),
          config_.has_twist_ ? vel_fp16_d_.get() : nullptr, stream_));
      } else {
        CHECK_CUDA_ERROR(transhead_post_ptr_->process(
          query_heatmap_score_fp16_d_.get(), query_labels_i64_d_.get(), heatmap_fp16_d_.get(),
          center_fp16_d_.get(), height_fp16_d_.get(), dim_fp16_d_.get(), rot_fp16_d_.get(),
          config_.has_twist_ ? vel_fp16_d_.get() : nullptr, stream_));
      }
    } else {
      if (query_labels_dtype_ == nvinfer1::DataType::kINT32) {
        CHECK_CUDA_ERROR(transhead_post_ptr_->process(
          query_heatmap_score_d_.get(), query_labels_i32_d_.get(), heatmap_d_.get(),
          center_d_.get(), height_d_.get(), dim_d_.get(), rot_d_.get(),
          config_.has_twist_ ? vel_d_.get() : nullptr, stream_));
      } else {
        CHECK_CUDA_ERROR(transhead_post_ptr_->process(
          query_heatmap_score_d_.get(), query_labels_i64_d_.get(), heatmap_d_.get(),
          center_d_.get(), height_d_.get(), dim_d_.get(), rot_d_.get(),
          config_.has_twist_ ? vel_d_.get() : nullptr, stream_));
      }
    }
  }

  // Shared: both heads produce score-filtered boxes in the same device format.
  const Box3D * device_boxes = (config_.detection_head_type_ == DetectionHeadType::CenterHead)
                                 ? centerhead_post_ptr_->device_boxes()
                                 : transhead_post_ptr_->device_boxes();
  const std::size_t num_boxes = (config_.detection_head_type_ == DetectionHeadType::CenterHead)
                                  ? centerhead_post_ptr_->num_boxes()
                                  : transhead_post_ptr_->num_boxes();

  det_boxes3d.resize(num_boxes);
  if (num_boxes == 0) {
    return true;
  }
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    det_boxes3d.data(), device_boxes, num_boxes * sizeof(Box3D), cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  return true;
}

}  //  namespace autoware::ptv3
