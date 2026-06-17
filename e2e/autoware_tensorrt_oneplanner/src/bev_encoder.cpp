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

#include "autoware/tensorrt_oneplanner/bev_encoder.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_oneplanner
{

using autoware::bevfusion::BEVFusionConfig;
using autoware::bevfusion::DensificationParam;
using autoware::tensorrt_common::NetworkIO;
using autoware::tensorrt_common::ProfileDims;
using autoware::tensorrt_common::Profiler;
using autoware::tensorrt_common::TrtCommon;
using autoware::tensorrt_common::TrtCommonConfig;

BevEncoder::BevEncoder(const BevEncoderParams & params, cudaStream_t stream)
: params_(params), stream_(stream)
{
  init_config();

  const DensificationParam densification_param(
    params_.densification_world_frame_id,
    static_cast<unsigned int>(params_.densification_num_past_frames));
  voxel_generator_ =
    std::make_unique<autoware::bevfusion::VoxelGenerator>(densification_param, *config_, stream_);
  preprocess_ = std::make_unique<autoware::bevfusion::PreprocessCuda>(*config_, stream_, true);

  points_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_->cloud_capacity_ * config_->num_point_feature_size_);
  voxel_features_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_->max_num_voxels_ * config_->max_points_per_voxel_ * config_->num_point_feature_size_);
  voxel_coords_d_ = autoware::cuda_utils::make_unique<std::int32_t[]>(3 * config_->max_num_voxels_);
  num_points_per_voxel_d_ =
    autoware::cuda_utils::make_unique<std::int32_t[]>(config_->max_num_voxels_);
  coords_axis_scratch_d_ =
    autoware::cuda_utils::make_unique<std::int32_t[]>(config_->max_num_voxels_);

  const int64_t grid_xy = static_cast<int64_t>(
    (params_.point_cloud_range[3] - params_.point_cloud_range[0]) / params_.voxel_size[0]);
  bev_size_ = grid_xy / params_.out_size_factor;
  bev_feature_size_ = static_cast<size_t>(params_.bev_feature_channels * bev_size_ * bev_size_);
  bev_feature_d_ = autoware::cuda_utils::make_unique<float[]>(bev_feature_size_);

  init_trt();
}

void BevEncoder::init_config()
{
  // Build a lidar-only BEVFusionConfig. All camera-related fields are zero / empty and the
  // detection postprocessing fields are unused because the encoder graph ends at the neck.
  config_ = std::make_unique<BEVFusionConfig>(
    /*class_size=*/0, params_.plugins_path,
    /*image_backbone_onnx_path=*/"", /*image_backbone_engine_path=*/"",
    /*image_backbone_trt_precision=*/"", params_.out_size_factor, params_.cloud_capacity,
    params_.max_points_per_voxel, params_.voxels_num, params_.point_cloud_range, params_.voxel_size,
    /*d_bound=*/std::vector<float>{1.0f, 166.2f, 1.4f},
    /*x_bound=*/
    std::vector<float>{
      params_.point_cloud_range[0], params_.point_cloud_range[3],
      params_.voxel_size[0] * static_cast<float>(params_.out_size_factor)},
    /*y_bound=*/
    std::vector<float>{
      params_.point_cloud_range[1], params_.point_cloud_range[4],
      params_.voxel_size[1] * static_cast<float>(params_.out_size_factor)},
    /*z_bound=*/std::vector<float>{-10.0f, 10.0f, 20.0f},
    /*num_cameras=*/0, /*raw_image_height=*/0, /*raw_image_width=*/0,
    /*img_aug_scale_x=*/0.0f, /*img_aug_scale_y=*/0.0f, /*roi_height=*/0, /*roi_width=*/0,
    /*features_height=*/0, /*features_width=*/0, /*num_depth_features=*/0,
    /*image_feature_channel=*/0, /*num_proposals=*/0,
    /*circle_nms_dist_threshold=*/0.0f, /*yaw_norm_thresholds=*/std::vector<double>{},
    /*score_thresholds=*/std::vector<float>{},
    /*distance_bin_upper_limits=*/std::vector<float>{},
    // 4-dim (x, y, z, time_lag) or 5-dim (+intensity) points; must match the
    // point dimension the OnePlanner checkpoint was trained on.
    params_.use_intensity);
}

void BevEncoder::init_trt()
{
  std::vector<NetworkIO> network_io;
  network_io.emplace_back(
    "voxels",
    nvinfer1::Dims{3, {-1, config_->max_points_per_voxel_, config_->num_point_feature_size_}});
  network_io.emplace_back("num_points_per_voxel", nvinfer1::Dims{1, {-1}});
  network_io.emplace_back("coors", nvinfer1::Dims{2, {-1, BEVFusionConfig::kNum3DCoords}});
  network_io.emplace_back(
    "bev_feature_map", nvinfer1::Dims{4, {1, params_.bev_feature_channels, bev_size_, bev_size_}});

  std::vector<ProfileDims> profile_dims;
  profile_dims.emplace_back(
    "voxels",
    nvinfer1::Dims{
      3,
      {config_->voxels_num_[0], config_->max_points_per_voxel_, config_->num_point_feature_size_}},
    nvinfer1::Dims{
      3,
      {config_->voxels_num_[1], config_->max_points_per_voxel_, config_->num_point_feature_size_}},
    nvinfer1::Dims{
      3,
      {config_->voxels_num_[2], config_->max_points_per_voxel_, config_->num_point_feature_size_}});
  profile_dims.emplace_back(
    "num_points_per_voxel", nvinfer1::Dims{1, {config_->voxels_num_[0]}},
    nvinfer1::Dims{1, {config_->voxels_num_[1]}}, nvinfer1::Dims{1, {config_->voxels_num_[2]}});
  profile_dims.emplace_back(
    "coors", nvinfer1::Dims{2, {config_->voxels_num_[0], BEVFusionConfig::kNum3DCoords}},
    nvinfer1::Dims{2, {config_->voxels_num_[1], BEVFusionConfig::kNum3DCoords}},
    nvinfer1::Dims{2, {config_->voxels_num_[2], BEVFusionConfig::kNum3DCoords}});

  auto network_io_ptr = std::make_unique<std::vector<NetworkIO>>(network_io);
  auto profile_dims_ptr = std::make_unique<std::vector<ProfileDims>>(profile_dims);

  const TrtCommonConfig trt_config(
    params_.onnx_path, params_.trt_precision, params_.engine_path, (1ULL << 32U));
  network_trt_ptr_ = std::make_unique<TrtCommon>(
    trt_config, std::make_shared<Profiler>(), std::vector<std::string>{params_.plugins_path});

  if (!network_trt_ptr_->setup(std::move(profile_dims_ptr), std::move(network_io_ptr))) {
    throw std::runtime_error("Failed to setup the OnePlanner BEV encoder TRT engine.");
  }

  network_trt_ptr_->setTensorAddress("voxels", voxel_features_d_.get());
  network_trt_ptr_->setTensorAddress("num_points_per_voxel", num_points_per_voxel_d_.get());
  network_trt_ptr_->setTensorAddress("coors", voxel_coords_d_.get());
  network_trt_ptr_->setTensorAddress("bev_feature_map", bev_feature_d_.get());
}

bool BevEncoder::encode(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & pointcloud_msg_ptr,
  const tf2_ros::Buffer & tf_buffer)
{
  const auto logger = rclcpp::get_logger("tensorrt_oneplanner");

  if (!voxel_generator_->enqueuePointCloud(pointcloud_msg_ptr, tf_buffer)) {
    return false;
  }

  // Clear device buffers reused across frames
  autoware::cuda_utils::clear_async(
    points_d_.get(), config_->cloud_capacity_ * config_->num_point_feature_size_, stream_);
  autoware::cuda_utils::clear_async(
    voxel_features_d_.get(),
    config_->max_num_voxels_ * config_->max_points_per_voxel_ * config_->num_point_feature_size_,
    stream_);
  autoware::cuda_utils::clear_async(voxel_coords_d_.get(), 3 * config_->max_num_voxels_, stream_);
  autoware::cuda_utils::clear_async(
    num_points_per_voxel_d_.get(), config_->max_num_voxels_, stream_);

  const auto num_points = voxel_generator_->generateSweepPoints(points_d_);
  if (num_points == 0) {
    RCLCPP_ERROR(logger, "Empty sweep points. Skipping inference.");
    return false;
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  auto num_voxels = static_cast<std::int64_t>(preprocess_->generateVoxels(
    points_d_.get(), num_points, voxel_features_d_.get(), voxel_coords_d_.get(),
    num_points_per_voxel_d_.get()));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (num_voxels < config_->min_num_voxels_) {
    RCLCPP_ERROR_STREAM(
      logger, "Too few voxels (" << num_voxels << ") for the optimization profile ("
                                 << config_->min_num_voxels_ << ")");
    return false;
  }
  if (num_voxels > config_->max_num_voxels_) {
    RCLCPP_WARN_STREAM(
      logger, "The actual number of voxels (" << num_voxels << ") exceeds the maximum ("
                                              << config_->max_num_voxels_ << "). Clipping.");
    num_voxels = config_->max_num_voxels_;
  }

  // autoware_bevfusion voxelization emits voxel coords as (z, y, x), but the
  // OnePlanner BEV encoder was trained/exported with (x, y, z) coords and
  // sparse_shape [X, Y, Z] = [1440, 1440, 41]. Swap the first and last of the
  // three coord columns so the Z index lands in the last column (size 41);
  // otherwise the first strided sparse conv drops every point ("points
  // vanished") because the X index (0..1439) is validated against the Z extent.
  //
  // Done fully on the device as three strided (column) copies queued on stream_
  // — no host round-trip and no synchronization — so it overlaps with the rest
  // of the pipeline and adds negligible latency.
  swap_coord_columns(num_voxels);

  network_trt_ptr_->setInputShape(
    "voxels", nvinfer1::Dims{
                3, {num_voxels, config_->max_points_per_voxel_, config_->num_point_feature_size_}});
  network_trt_ptr_->setInputShape("num_points_per_voxel", nvinfer1::Dims{1, {num_voxels}});
  network_trt_ptr_->setInputShape(
    "coors", nvinfer1::Dims{2, {num_voxels, BEVFusionConfig::kNum3DCoords}});

  if (!network_trt_ptr_->enqueueV3(stream_)) {
    RCLCPP_ERROR(logger, "Failed to enqueue the BEV encoder engine.");
    return false;
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return true;
}

void BevEncoder::swap_coord_columns(int64_t num_voxels)
{
  // The buffer is row-major [num_voxels, 3] int32: row pitch = 3 elements, the
  // first column is at offset 0 and the last at offset 2. Use strided 2D copies
  // (1 element wide, num_voxels tall) to exchange the two columns via a scratch
  // column, entirely on stream_.
  constexpr size_t kElem = sizeof(std::int32_t);
  constexpr size_t kRowPitch = 3 * kElem;
  const auto height = static_cast<size_t>(num_voxels);
  std::int32_t * coords = voxel_coords_d_.get();
  std::int32_t * scratch = coords_axis_scratch_d_.get();

  // scratch <- column 0
  CHECK_CUDA_ERROR(cudaMemcpy2DAsync(
    scratch, kElem, coords, kRowPitch, kElem, height, cudaMemcpyDeviceToDevice, stream_));
  // column 0 <- column 2
  CHECK_CUDA_ERROR(cudaMemcpy2DAsync(
    coords, kRowPitch, coords + 2, kRowPitch, kElem, height, cudaMemcpyDeviceToDevice, stream_));
  // column 2 <- scratch
  CHECK_CUDA_ERROR(cudaMemcpy2DAsync(
    coords + 2, kRowPitch, scratch, kElem, kElem, height, cudaMemcpyDeviceToDevice, stream_));
}

}  // namespace autoware::tensorrt_oneplanner
