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

#ifndef AUTOWARE__PTV3__PTV3_CONFIG_HPP_
#define AUTOWARE__PTV3__PTV3_CONFIG_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ptv3
{

enum class SourceReconstruction {
  NONE,
  PARTIAL,
  FULL,
};

class PTv3Config
{
public:
  PTv3Config(
    const bool use_seg3d_head, const bool use_det3d_head, const std::string & plugins_path,
    const std::int64_t cloud_capacity, const std::vector<std::int64_t> & voxels_num,
    const std::vector<float> & point_cloud_range, const std::vector<float> & voxel_size,
    const std::vector<std::string> & segmentation_class_names = {},
    const std::vector<std::string> & serialization_orders = {},
    const std::vector<std::int64_t> & pooling_strides = {},
    const std::vector<std::int64_t> & palette = {},
    const float filter_class_probability_threshold = {},
    const std::vector<std::string> & filter_classes = {},
    const std::string & filter_output_format = {}, const std::string & source_reconstruction = {},
    const std::vector<std::string> & detection_class_names = {},
    const std::vector<float> & bbox_voxel_size = {},
    const std::vector<float> & distance_bin_upper_limits = {},
    const std::vector<float> & detection_score_thresholds = {},
    const std::vector<float> & yaw_norm_thresholds = {}, const bool has_twist = {},
    std::size_t num_proposals = {}, const std::vector<float> & post_center_range = {})
  : use_seg3d_head_(use_seg3d_head), use_det3d_head_(use_det3d_head)
  {
    plugins_path_ = plugins_path;

    cloud_capacity_ = cloud_capacity;

    if (!use_seg3d_head_ && !use_det3d_head_) {
      throw std::runtime_error(
        "At least one of segmentation3d.use_head or detection3d.use_head must be true.");
    }

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

    grid_x_size_ = static_cast<std::int64_t>((max_x_range_ - min_x_range_) / voxel_x_size_);
    grid_y_size_ = static_cast<std::int64_t>((max_y_range_ - min_y_range_) / voxel_y_size_);
    grid_z_size_ = static_cast<std::int64_t>((max_z_range_ - min_z_range_) / voxel_z_size_);
    auto max_grid_size = std::max({grid_x_size_, grid_y_size_, grid_z_size_});
    serialization_depth_ =
      static_cast<std::int32_t>(std::ceil(std::log2(static_cast<float>(max_grid_size))));
    auto max_voxels_depth =
      static_cast<std::int32_t>(std::ceil(std::log2(static_cast<float>(max_num_voxels_))));
    if (serialization_depth_ * 3 + max_voxels_depth >= 64) {
      throw std::runtime_error("Serialization depth is too large");
    }

    use_64bit_hash_ =
      grid_x_size_ * grid_y_size_ * grid_z_size_ > std::numeric_limits<std::uint32_t>::max();

    serialization_orders_ = validate_serialization_orders(serialization_orders);
    pooling_strides_ = validate_pooling_strides(pooling_strides);

    if (use_seg3d_head_) {
      segmentation_class_names_ = segmentation_class_names;
      colors_rgb_ = make_palette(segmentation_class_names_, palette);
      for (auto & class_name : segmentation_class_names_) {
        std::transform(
          class_name.begin(), class_name.end(), class_name.begin(),
          [](unsigned char c) { return std::tolower(c); });
      }
      filter_class_probability_threshold_ = filter_class_probability_threshold;
      filter_class_indices_ = make_filter_class_indices(segmentation_class_names_, filter_classes);
      filter_output_format_ = filter_output_format;
      source_reconstruction_ = parse_source_reconstruction(source_reconstruction);
    }

    if (use_det3d_head_) {
      if (detection_class_names.empty()) {
        throw std::runtime_error("detection_class_names must not be empty when use_det3d_head.");
      }
      if (bbox_voxel_size.size() != 3) {
        throw std::runtime_error("bbox_voxel_size must contain 3 elements.");
      }
      if (bbox_voxel_size[0] <= 0.0F || bbox_voxel_size[1] <= 0.0F || bbox_voxel_size[2] <= 0.0F) {
        throw std::runtime_error("bbox_voxel_size values must be positive.");
      }
      if (distance_bin_upper_limits.empty()) {
        throw std::runtime_error("distance_bin_upper_limits must not be empty.");
      }
      if (!std::is_sorted(distance_bin_upper_limits.begin(), distance_bin_upper_limits.end())) {
        throw std::runtime_error("distance_bin_upper_limits must be sorted.");
      }
      if (distance_bin_upper_limits.front() <= 0.0F) {
        throw std::runtime_error("distance_bin_upper_limits values must be positive.");
      }
      if (
        detection_score_thresholds.size() !=
        distance_bin_upper_limits.size() * detection_class_names.size()) {
        throw std::runtime_error(
          "detection_score_thresholds size must match distance bins x detection classes.");
      }
      if (!std::all_of(
            detection_score_thresholds.begin(), detection_score_thresholds.end(),
            [](float value) { return value >= 0.0F && value <= 1.0F; })) {
        throw std::runtime_error("detection_score_thresholds values must be between 0 and 1.");
      }
      if (yaw_norm_thresholds.size() != detection_class_names.size()) {
        throw std::runtime_error("yaw_norm_thresholds size must match detection class_names.");
      }
      if (!std::all_of(yaw_norm_thresholds.begin(), yaw_norm_thresholds.end(), [](float value) {
            return value >= 0.0F && value <= 1.0F;
          })) {
        throw std::runtime_error("yaw_norm_thresholds values must be between 0 and 1.");
      }

      detection_class_names_ = detection_class_names;
      bbox_voxel_x_size_ = bbox_voxel_size[0];
      bbox_voxel_y_size_ = bbox_voxel_size[1];
      distance_bin_upper_limits_ = distance_bin_upper_limits;
      detection_score_thresholds_ = detection_score_thresholds;
      yaw_norm_thresholds_ = yaw_norm_thresholds;
      has_twist_ = has_twist;

      const auto bbox_grid_x_size =
        static_cast<std::size_t>((max_x_range_ - min_x_range_) / bbox_voxel_x_size_);
      const auto bbox_grid_y_size =
        static_cast<std::size_t>((max_y_range_ - min_y_range_) / bbox_voxel_y_size_);
      if (bbox_grid_x_size == 0 || bbox_grid_y_size == 0) {
        throw std::runtime_error("bbox_voxel_size produces an empty detection grid.");
      }
      det_grid_x_size_ = bbox_grid_x_size;
      det_grid_y_size_ = bbox_grid_y_size;

      if (num_proposals == 0) {
        throw std::runtime_error("num_proposals must be positive.");
      }
      if (post_center_range.size() != 6) {
        throw std::runtime_error("post_center_range must contain 6 elements.");
      }
      if (
        post_center_range[0] >= post_center_range[3] ||
        post_center_range[1] >= post_center_range[4] ||
        post_center_range[2] >= post_center_range[5]) {
        throw std::runtime_error(
          "post_center_range minimum values must be smaller than maximum values.");
      }
      num_proposals_ = num_proposals;
      post_center_range_ = post_center_range;
    }
  }

  static SourceReconstruction parse_source_reconstruction(const std::string & value)
  {
    if (value == "none") {
      return SourceReconstruction::NONE;
    }
    if (value == "partial") {
      return SourceReconstruction::PARTIAL;
    }
    if (value == "full") {
      return SourceReconstruction::FULL;
    }
    throw std::runtime_error("source_reconstruction must be one of: 'none', 'partial', or 'full'.");
  }

  static std::vector<std::uint32_t> make_filter_class_indices(
    const std::vector<std::string> & class_names, const std::vector<std::string> & filter_classes)
  {
    std::vector<std::uint32_t> indices;
    for (const auto & filter_class : filter_classes) {
      auto it = std::find(class_names.begin(), class_names.end(), filter_class);
      if (it == class_names.end()) {
        throw std::runtime_error("Filter class '" + filter_class + "' not found in class names.");
      }
      indices.push_back(static_cast<std::uint32_t>(std::distance(class_names.begin(), it)));
    }
    return indices;
  }

  static std::vector<float> make_palette(
    const std::vector<std::string> & class_names, const std::vector<std::int64_t> & palette)
  {
    if (palette.size() % 3 != 0) {
      throw std::runtime_error("Palette size must be a multiple of 3.");
    }
    if (palette.size() != class_names.size() * 3) {
      throw std::runtime_error("Palette size does not match class names size.");
    }

    std::vector<float> colors;
    colors.reserve(class_names.size());
    for (size_t i = 0; i < palette.size(); i += 3) {
      const auto r = palette[i];
      const auto g = palette[i + 1];
      const auto b = palette[i + 2];
      if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
        throw std::runtime_error("Color values must be within 0-255 range.");
      }

      const std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16u) |
                                (static_cast<std::uint32_t>(g) << 8u) |
                                static_cast<std::uint32_t>(b);
      float rgb_float = 0.0f;
      memcpy(&rgb_float, &rgb, sizeof(rgb_float));
      colors.push_back(rgb_float);
    }
    return colors;
  }

  static std::vector<std::string> validate_serialization_orders(
    const std::vector<std::string> & serialization_orders)
  {
    if (serialization_orders.empty()) {
      throw std::runtime_error("serialization_orders must not be empty.");
    }
    if (
      serialization_orders.size() != 2 || serialization_orders[0] != "z" ||
      serialization_orders[1] != "z-trans") {
      throw std::runtime_error(
        "The current PTv3 preprocessing path supports serialization_orders: ['z', 'z-trans'].");
    }
    return serialization_orders;
  }

  static std::vector<std::int64_t> validate_pooling_strides(
    const std::vector<std::int64_t> & pooling_strides)
  {
    if (pooling_strides.empty()) {
      throw std::runtime_error("pooling_strides must not be empty.");
    }
    for (const auto stride : pooling_strides) {
      if (stride < 1 || (stride & (stride - 1)) != 0) {
        throw std::runtime_error("Each pooling stride must be a positive power of two.");
      }
    }
    return pooling_strides;
  }

  // CUDA parameters
  const std::uint32_t threads_per_block_{256};  // threads number for a block

  // TensorRT parameters
  std::string plugins_path_;

  // Head selection
  bool use_seg3d_head_;
  bool use_det3d_head_;

  // Preprocess parameters
  bool use_64bit_hash_{};
  std::int32_t serialization_depth_{};

  ///// NETWORK PARAMETERS /////

  // Backbone
  std::vector<std::string> segmentation_class_names_;
  std::vector<std::string> serialization_orders_;
  std::vector<std::int64_t> pooling_strides_;

  // Segmentation head
  std::vector<float> colors_rgb_;
  float filter_class_probability_threshold_{};
  std::vector<std::uint32_t> filter_class_indices_;
  std::string filter_output_format_;
  SourceReconstruction source_reconstruction_{SourceReconstruction::NONE};

  // Detection head
  std::vector<std::string> detection_class_names_;
  float bbox_voxel_x_size_{};
  float bbox_voxel_y_size_{};
  bool has_twist_{};
  std::size_t num_proposals_{};
  std::vector<float> post_center_range_;
  std::vector<float> distance_bin_upper_limits_;
  std::vector<float> detection_score_thresholds_;
  std::vector<float> yaw_norm_thresholds_;
  std::size_t det_grid_x_size_{};
  std::size_t det_grid_y_size_{};

  // Common network parameters
  std::int64_t cloud_capacity_{};
  std::int64_t min_num_voxels_{};
  std::int64_t max_num_voxels_{};
  const std::int64_t num_point_feature_size_{4};  // x, y, z, intensity
  const std::int64_t backbone_feat_dim_{64};      // backbone output feature dimension

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

  ///// RUNTIME DIMENSIONS /////
  std::array<std::int64_t, 3> voxels_num_{};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PTV3_CONFIG_HPP_
