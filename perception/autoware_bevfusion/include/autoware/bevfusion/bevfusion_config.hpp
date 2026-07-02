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

#ifndef AUTOWARE__BEVFUSION__BEVFUSION_CONFIG_HPP_
#define AUTOWARE__BEVFUSION__BEVFUSION_CONFIG_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::bevfusion
{

class BEVFusionConfig
{
public:
  BEVFusionConfig(
    const std::size_t class_size, const std::string & plugins_path,
    const std::string & image_backbone_onnx_path, const std::string & image_backbone_engine_path,
    const std::string & image_backbone_trt_precision, const std::int64_t out_size_factor,
    const std::int64_t cloud_capacity, const std::int64_t max_points_per_voxel,
    const std::vector<std::int64_t> & voxels_num, const std::vector<float> & point_cloud_range,
    const std::vector<float> & voxel_size, const std::vector<float> & d_bound,
    const std::vector<float> & x_bound, const std::vector<float> & y_bound,
    const std::vector<float> & z_bound, const std::int64_t num_cameras,
    const std::int64_t raw_image_height, const std::int64_t raw_image_width,
    const float img_aug_scale_x, const float img_aug_scale_y, const std::int64_t roi_height,
    const std::int64_t roi_width, const std::int64_t features_height,
    const std::int64_t features_width, const std::int64_t num_depth_features,
    const std::int64_t image_feature_channel, const std::int64_t num_proposals,
    const float circle_nms_dist_threshold, const std::vector<double> & yaw_norm_thresholds,
    const std::vector<float> & score_thresholds,
    const std::vector<float> & distance_bin_upper_limits, const bool use_intensity)
  {
    // Derive sensor_fusion from image backbone parameters
    // All three must be empty OR all three must be non-empty
    const bool all_empty = image_backbone_onnx_path.empty() && image_backbone_engine_path.empty() &&
                           image_backbone_trt_precision.empty();
    const bool all_non_empty = !image_backbone_onnx_path.empty() &&
                               !image_backbone_engine_path.empty() &&
                               !image_backbone_trt_precision.empty();

    if (!all_empty && !all_non_empty) {
      throw std::invalid_argument(
        "Image backbone parameters must be either all empty (lidar-only mode) or all non-empty "
        "(fusion mode). Got: image_backbone_onnx_path='" +
        image_backbone_onnx_path + "', image_backbone_engine_path='" + image_backbone_engine_path +
        "', image_backbone_trt_precision='" + image_backbone_trt_precision + "'");
    }

    sensor_fusion_ = all_non_empty;

    if (use_intensity) {
      // x, y, z, intensity, timestamp_lag
      num_point_feature_size_ = 5;
    } else {
      // x, y, z, timestamp_lag
      num_point_feature_size_ = 4;
    }
    use_intensity_ = use_intensity;
    plugins_path_ = plugins_path;

    out_size_factor_ = out_size_factor;

    cloud_capacity_ = cloud_capacity;
    max_points_per_voxel_ = max_points_per_voxel;

    if (voxels_num.size() == 3) {
      min_num_voxels_ = voxels_num[0];
      max_num_voxels_ = voxels_num[2];

      voxels_num_[0] = voxels_num[0];
      voxels_num_[1] = voxels_num[1];
      voxels_num_[2] = voxels_num[2];
    }
    if (point_cloud_range.size() == 6) {
      min_x_range_ = point_cloud_range[0];
      min_y_range_ = point_cloud_range[1];
      min_z_range_ = point_cloud_range[2];
      max_x_range_ = point_cloud_range[3];
      max_y_range_ = point_cloud_range[4];
      max_z_range_ = point_cloud_range[5];
    }
    if (voxel_size.size() == 3) {
      voxel_x_size_ = voxel_size[0];
      voxel_y_size_ = voxel_size[1];
      voxel_z_size_ = voxel_size[2];
    }
    if (d_bound.size() == 3 && x_bound.size() == 3 && y_bound.size() == 3 && z_bound.size() == 3) {
      d_bound_ = d_bound;
      x_bound_ = x_bound;
      y_bound_ = y_bound;
      z_bound_ = z_bound;
    }

    num_cameras_ = num_cameras;
    raw_image_height_ = raw_image_height;
    raw_image_width_ = raw_image_width;
    img_aug_scale_x_ = img_aug_scale_x;
    img_aug_scale_y_ = img_aug_scale_y;
    roi_height_ = roi_height;
    roi_width_ = roi_width;
    features_height_ = features_height;
    features_width_ = features_width;
    num_depth_features_ = num_depth_features;
    image_feature_channel_ = image_feature_channel;
    resized_height_ = raw_image_height_ * img_aug_scale_y_;
    resized_width_ = raw_image_width_ * img_aug_scale_x_;
    num_classes_ = class_size;

    if (num_proposals > 0) {
      num_proposals_ = num_proposals;
    }
    // score_upper_bounds must be sorted in ascending order, raise an error if not
    if (!std::is_sorted(distance_bin_upper_limits.begin(), distance_bin_upper_limits.end())) {
      throw std::invalid_argument("distance_bin_upper_limits must be sorted in ascending order");
    }
    distance_bin_upper_limits_ = distance_bin_upper_limits;
    for (auto & distance_bin_upper_limit : distance_bin_upper_limits_) {
      // Note: Square the distance bin upper limit to get the radial distance to skip the sqrtf
      // operation
      distance_bin_upper_limit = distance_bin_upper_limit * distance_bin_upper_limit;
    }

    // score_thresholds must have the size of score_upper_bounds * class_size
    if (score_thresholds.size() != distance_bin_upper_limits_.size() * num_classes_) {
      throw std::invalid_argument(
        "score_thresholds must have the size of distance_bin_upper_limits * class_size");
    }
    score_thresholds_ = score_thresholds;
    for (auto & score_threshold : score_thresholds_) {
      score_threshold = (score_threshold >= 0.f && score_threshold < 1.f) ? score_threshold : 0.f;
    }

    if (circle_nms_dist_threshold > 0.0) {
      circle_nms_dist_threshold_ = circle_nms_dist_threshold;
    }
    yaw_norm_thresholds_ =
      std::vector<float>(yaw_norm_thresholds.begin(), yaw_norm_thresholds.end());
    for (auto & yaw_norm_threshold : yaw_norm_thresholds_) {
      yaw_norm_threshold =
        (yaw_norm_threshold >= 0.0 && yaw_norm_threshold < 1.0) ? yaw_norm_threshold : 0.0;
    }
    grid_x_size_ = static_cast<std::int64_t>((max_x_range_ - min_x_range_) / voxel_x_size_);
    grid_y_size_ = static_cast<std::int64_t>((max_y_range_ - min_y_range_) / voxel_y_size_);
    grid_z_size_ = static_cast<std::int64_t>((max_z_range_ - min_z_range_) / voxel_z_size_);
  }

  ///// MODALITY /////
  bool sensor_fusion_{};
  bool use_intensity_{false};

  // CUDA parameters
  const std::uint32_t threads_per_block_{256};  // threads number for a block

  // TensorRT parameters
  std::string plugins_path_{};

  // Constants
  static constexpr std::int64_t kTransformMatrixDim = 4;  // 4x4 transformation matrix dimension
  static constexpr std::int64_t kNumRGBChannels = 3;      // RGB color channels
  static constexpr std::int64_t kNum3DCoords = 3;         // 3D coordinates (x, y, z)

  ///// NETWORK PARAMETERS /////

  // Common network parameters
  std::int64_t out_size_factor_{};

  std::int64_t cloud_capacity_{};
  std::int64_t min_num_voxels_{};
  std::int64_t max_num_voxels_{};
  std::int64_t max_points_per_voxel_;

  std::int64_t num_point_feature_size_{4};  // x, y, z, timestamp_lag

  // Pointcloud range in meters
  float min_x_range_{};
  float max_x_range_{};
  float min_y_range_{};
  float max_y_range_{};
  float min_z_range_{};
  float max_z_range_{};

  // Voxel size in meters
  float voxel_x_size_{};
  float voxel_y_size_{};
  float voxel_z_size_{};

  // Grid size
  std::int64_t grid_x_size_{};
  std::int64_t grid_y_size_{};
  std::int64_t grid_z_size_{};

  // Camera branch parameters
  std::vector<float> d_bound_{};
  std::vector<float> x_bound_{};
  std::vector<float> y_bound_{};
  std::vector<float> z_bound_{};

  std::int64_t num_cameras_{};
  std::int64_t raw_image_height_{};
  std::int64_t raw_image_width_{};

  float img_aug_scale_x_{};
  float img_aug_scale_y_{};

  std::int64_t roi_height_{};
  std::int64_t roi_width_{};

  std::int64_t resized_height_{};
  std::int64_t resized_width_{};

  std::int64_t features_height_{};
  std::int64_t features_width_{};
  std::int64_t num_depth_features_{};
  std::int64_t image_feature_channel_{256};  // Image feature dimension

  // Head parameters
  std::int64_t num_proposals_{};
  std::size_t num_classes_{5};

  // Post processing parameters

  // the score threshold for classification
  std::vector<float> distance_bin_upper_limits_{};
  std::vector<float> score_thresholds_{};

  float circle_nms_dist_threshold_{};
  std::vector<float> yaw_norm_thresholds_{};
  // the detected boxes result decode by (x, y, z, w, l, h, yaw, vx, vy)
  const std::int64_t num_box_values_{10};

  ///// RUNTIME DIMENSIONS /////
  std::array<std::int64_t, 3> voxels_num_{};

  ///// SPARSE trainStation/DDS REMOVAL /////
  // When true, the sparse engine was exported with the 4 down-sample GetIndicePairsImplicitGemm
  // nodes removed (their rulebooks exposed as graph inputs). The runtime then precomputes those
  // rulebooks from the voxel coordinates and binds them before inference. Plain member (set by the
  // node from a ROS param) to avoid extending the large positional constructor.
  // See BEVFusion_spconv_DDS_optimization.md (Slice 2b).
  bool sparse_remove_trainstation_{false};
  // Upper bound on active out-voxels per down-sample stage; must match the plugin's
  // out_indices_num_limit_ and the TensorRT profile max for the rulebook inputs.
  std::int64_t sparse_out_indices_num_limit_{256000};
  // Voxelizer coords order: true if `coors` is [z,y,x] (legacy Autoware graph-input contract) and
  // must be flipped to [x,y,z] before spconv. Matches the AWML export coors_contract.
  bool sparse_coors_is_zyx_{true};
};

}  // namespace autoware::bevfusion

#endif  // AUTOWARE__BEVFUSION__BEVFUSION_CONFIG_HPP_
