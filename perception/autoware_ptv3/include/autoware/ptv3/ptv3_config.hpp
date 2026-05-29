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
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ptv3
{

/** @brief How voxel labels are mapped back to points. */
enum class SourceReconstruction {
  NONE,
  PARTIAL,
  FULL,
};

/** @brief Detection head layouts supported by this package. */
enum class DetectionHeadType { CenterHead, TransHead };

/**
 * @brief Parse a detection head type string.
 *
 * @param s Expected value is `centerhead` or `transhead`.
 * @return Parsed detection head type.
 * @throws std::runtime_error if the value is not supported.
 */
inline DetectionHeadType parse_detection_head_type(const std::string & s)
{
  if (s == "center_head") return DetectionHeadType::CenterHead;
  if (s == "trans_head") return DetectionHeadType::TransHead;
  throw std::runtime_error(
    "Unsupported detection_head_type='" + s + "'. Expected 'center_head' or 'trans_head'.");
}

/**
 * @brief Runtime settings shared by preprocessing, TensorRT, and postprocessing.
 *
 * The constructor validates shape-sensitive values before CUDA buffers are allocated.
 */
class PTv3Config
{
public:
  /**
   * @brief Build a checked runtime config from ROS and model package parameters.
   *
   * @param plugins_path Path to the TensorRT plugin shared library.
   * @param cloud_capacity Maximum number of points accepted from one input cloud.
   * @param voxels_num Voxel count profile in `[min, opt, max]` order.
   * @param point_cloud_range Crop range in `[x_min, y_min, z_min, x_max, y_max, z_max]` order.
   * @param voxel_size Backbone voxel size in `[x, y, z]` order.
   * @param class_names Segmentation class names.
   * @param palette Flat RGB palette in `[r0, g0, b0, r1, g1, b1]` order.
   * @param filter_class_probability_threshold Threshold for dropping configured filter classes.
   * @param filter_classes Segmentation classes excluded from the filtered cloud above threshold.
   * @param filter_output_format Output point format for the filtered cloud.
   * @param source_reconstruction One of `none`, `partial`, or `full`.
   * @param use_seg3d_head Enable the segmentation head.
   * @param use_det3d_head Enable the detection head.
   * @param detection_class_names Detection class names. Required when detection is enabled.
   * @param bbox_voxel_size Detection BEV voxel size in `[x, y, z]` order.
   * @param bbox_downsample_factor Downsample factor from BEV grid to detection output grid.
   * @param distance_bin_upper_limits Sorted distance bins for score thresholds.
   * @param detection_score_thresholds Per-class thresholds for each distance bin.
   * @param yaw_norm_thresholds Per-class yaw vector norm thresholds.
   * @param has_twist Whether the detection head exports velocity tensors.
   * @param has_variance Whether the detection head exports variance tensors.
   * @param detection_head_type `centerhead` or `transhead`.
   * @param num_proposals Number of query proposals for TransHead.
   * @param post_center_range TransHead range filter in `[x_min, y_min, z_min, x_max, y_max, z_max]`
   * order.
   * @throws std::runtime_error if the configuration is inconsistent.
   */
  PTv3Config(
    const std::string & plugins_path, const std::int64_t cloud_capacity,
    const std::vector<std::int64_t> & voxels_num, const std::vector<float> & point_cloud_range,
    const std::vector<float> & voxel_size, const std::vector<std::string> & class_names,
    const std::vector<std::int64_t> & palette, const float filter_class_probability_threshold,
    const std::vector<std::string> & filter_classes, const std::string & filter_output_format,
    const std::string & source_reconstruction,
    // Head selection flags.
    bool use_seg3d_head, bool use_det3d_head,
    // Detection parameters are checked only when the detection head is enabled.
    const std::vector<std::string> & detection_class_names = {},
    const std::vector<float> & bbox_voxel_size = {0.64f, 0.64f, 8.0f},
    std::size_t bbox_downsample_factor = 1,
    const std::vector<float> & distance_bin_upper_limits = {},
    const std::vector<float> & detection_score_thresholds = {},
    const std::vector<float> & yaw_norm_thresholds = {}, bool has_twist = false,
    bool has_variance = false, const std::string & detection_head_type = "trans_head",
    std::size_t num_proposals = 0, const std::vector<float> & post_center_range = {})
  {
    plugins_path_ = plugins_path;
    cloud_capacity_ = cloud_capacity;
    use_seg3d_head_ = use_seg3d_head;
    use_det3d_head_ = use_det3d_head;

    if (!use_seg3d_head_ && !use_det3d_head_) {
      throw std::runtime_error(
        "At least one of segmentation3d.use_head or detection3d.use_head must be true.");
    }
    if (cloud_capacity <= 0) {
      throw std::runtime_error("cloud_capacity must be positive.");
    }
    if (cloud_capacity > std::numeric_limits<int>::max()) {
      throw std::runtime_error("cloud_capacity exceeds the CUDA preprocessing index range.");
    }

    if (voxels_num.size() != 3) {
      throw std::runtime_error("voxels_num must contain exactly 3 elements.");
    }
    if (voxels_num[0] <= 0 || voxels_num[1] <= 0 || voxels_num[2] <= 0) {
      throw std::runtime_error("voxels_num values must be positive.");
    }
    if (voxels_num[0] > voxels_num[1] || voxels_num[1] > voxels_num[2]) {
      throw std::runtime_error("voxels_num must be ordered as [min, opt, max].");
    }
    if (voxels_num[2] > std::numeric_limits<int>::max()) {
      throw std::runtime_error("voxels_num max exceeds the CUDA preprocessing index range.");
    }
    min_num_voxels_ = voxels_num[0];
    max_num_voxels_ = voxels_num[2];
    voxels_num_[0] = voxels_num[0];
    voxels_num_[1] = voxels_num[1];
    voxels_num_[2] = voxels_num[2];

    if (point_cloud_range.size() != 6) {
      throw std::runtime_error("point_cloud_range must contain exactly 6 elements.");
    }
    min_x_range_ = point_cloud_range[0];
    min_y_range_ = point_cloud_range[1];
    min_z_range_ = point_cloud_range[2];
    max_x_range_ = point_cloud_range[3];
    max_y_range_ = point_cloud_range[4];
    max_z_range_ = point_cloud_range[5];
    if (
      min_x_range_ >= max_x_range_ || min_y_range_ >= max_y_range_ ||
      min_z_range_ >= max_z_range_) {
      throw std::runtime_error(
        "point_cloud_range minimum values must be smaller than maximum values.");
    }

    if (voxel_size.size() != 3) {
      throw std::runtime_error("voxel_size must contain exactly 3 elements.");
    }
    voxel_x_size_ = voxel_size[0];
    voxel_y_size_ = voxel_size[1];
    voxel_z_size_ = voxel_size[2];
    if (voxel_x_size_ <= 0.0F || voxel_y_size_ <= 0.0F || voxel_z_size_ <= 0.0F) {
      throw std::runtime_error("voxel_size values must be positive.");
    }

    grid_x_size_ = static_cast<std::int64_t>((max_x_range_ - min_x_range_) / voxel_x_size_);
    grid_y_size_ = static_cast<std::int64_t>((max_y_range_ - min_y_range_) / voxel_y_size_);
    grid_z_size_ = static_cast<std::int64_t>((max_z_range_ - min_z_range_) / voxel_z_size_);
    if (grid_x_size_ <= 0 || grid_y_size_ <= 0 || grid_z_size_ <= 0) {
      throw std::runtime_error("voxel_size and point_cloud_range produce an empty voxel grid.");
    }
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

    class_names_ = class_names;
    colors_rgb_ = make_palette(class_names_, palette);
    if (use_seg3d_head_ && class_names_.empty()) {
      throw std::runtime_error("segmentation3d.class_names must not be empty when use_seg3d_head.");
    }

    for (auto & class_name : class_names_) {
      std::transform(class_name.begin(), class_name.end(), class_name.begin(), [](unsigned char c) {
        return std::tolower(c);
      });
    }
    filter_class_probability_threshold_ = filter_class_probability_threshold;
    if (filter_class_probability_threshold_ < 0.0F || filter_class_probability_threshold_ > 1.0F) {
      throw std::runtime_error(
        "segmentation3d.filter.class_probability_threshold must be between 0 and 1.");
    }
    filter_class_indices_ = make_filter_class_indices(class_names_, filter_classes);
    filter_output_format_ = filter_output_format;
    source_reconstruction_ = parse_source_reconstruction(source_reconstruction);

    // Detection configuration.
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
      if (bbox_downsample_factor == 0) {
        throw std::runtime_error("bbox_downsample_factor must be positive.");
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
      bbox_voxel_z_size_ = bbox_voxel_size[2];
      bbox_downsample_factor_ = bbox_downsample_factor;
      distance_bin_upper_limits_ = distance_bin_upper_limits;
      detection_score_thresholds_ = detection_score_thresholds;
      yaw_norm_thresholds_ = yaw_norm_thresholds;
      has_twist_ = has_twist;
      has_variance_ = has_variance;
      detection_head_type_ = parse_detection_head_type(detection_head_type);
      if (detection_head_type_ == DetectionHeadType::TransHead && has_variance_) {
        throw std::runtime_error("has_variance is not supported for TransHead.");
      }

      if (has_variance_) {
        head_out_reg_size_ = 4;
        head_out_height_size_ = 2;
        head_out_dim_size_ = 6;
        head_out_rot_size_ = 4;
        head_out_vel_size_ = has_twist_ ? 4 : 0;
      } else {
        head_out_vel_size_ = has_twist_ ? 2 : 0;
      }

      bbox_grid_x_size_ =
        static_cast<std::size_t>((max_x_range_ - min_x_range_) / bbox_voxel_x_size_);
      bbox_grid_y_size_ =
        static_cast<std::size_t>((max_y_range_ - min_y_range_) / bbox_voxel_y_size_);
      if (bbox_grid_x_size_ == 0 || bbox_grid_y_size_ == 0) {
        throw std::runtime_error("bbox_voxel_size produces an empty detection grid.");
      }
      if (
        bbox_grid_x_size_ % bbox_downsample_factor_ != 0 ||
        bbox_grid_y_size_ % bbox_downsample_factor_ != 0) {
        throw std::runtime_error("bbox grid size must be divisible by bbox_downsample_factor.");
      }
      det_grid_x_size_ = bbox_grid_x_size_ / bbox_downsample_factor_;
      det_grid_y_size_ = bbox_grid_y_size_ / bbox_downsample_factor_;

      if (detection_head_type_ == DetectionHeadType::TransHead) {
        if (num_proposals == 0) {
          throw std::runtime_error("num_proposals must be positive for TransHead.");
        }
        if (post_center_range.size() != 6) {
          throw std::runtime_error("post_center_range must contain 6 elements for TransHead.");
        }
        if (
          post_center_range[0] >= post_center_range[3] ||
          post_center_range[1] >= post_center_range[4] ||
          post_center_range[2] >= post_center_range[5]) {
          throw std::runtime_error(
            "post_center_range minimum values must be smaller than maximum values.");
        }
      }
      num_proposals_ = num_proposals;
      post_center_range_ = post_center_range;
    }
  }

  /**
   * @brief Parse source reconstruction mode from the parameter string.
   *
   * @param value Expected value is `none`, `partial`, or `full`.
   * @return Parsed reconstruction mode.
   * @throws std::runtime_error if the value is not supported.
   */
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

  /**
   * @brief Convert filter class names to dense segmentation indices.
   *
   * @param class_names Ordered segmentation class names.
   * @param filter_classes Class names excluded from the filtered cloud above threshold.
   * @return Indices into class_names for each filter class.
   * @throws std::runtime_error if a filter class is missing.
   */
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

  /**
   * @brief Pack the visualization palette into float RGB values used by PointCloud2.
   *
   * @param class_names Segmentation class names used to validate palette size.
   * @param palette Flat RGB palette with channel values in the 0 to 255 range.
   * @return Packed float RGB values, one value per class.
   * @throws std::runtime_error if the palette shape or channel values are invalid.
   */
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

  // CUDA parameters
  const std::uint32_t threads_per_block_{256};

  // TensorRT parameters
  std::string plugins_path_;

  // Head selection
  bool use_seg3d_head_{true};
  bool use_det3d_head_{false};

  // Preprocess parameters
  bool use_64bit_hash_{};
  std::int32_t serialization_depth_{};

  // Segmentation head.
  std::vector<std::string> class_names_;
  std::vector<float> colors_rgb_;
  float filter_class_probability_threshold_{};
  std::vector<std::uint32_t> filter_class_indices_;
  std::string filter_output_format_;
  SourceReconstruction source_reconstruction_{SourceReconstruction::NONE};

  // Detection head.
  std::vector<std::string> detection_class_names_;
  float bbox_voxel_x_size_{0.64f};
  float bbox_voxel_y_size_{0.64f};
  float bbox_voxel_z_size_{8.0f};
  std::size_t bbox_downsample_factor_{1};
  bool has_twist_{false};
  bool has_variance_{false};
  DetectionHeadType detection_head_type_{DetectionHeadType::CenterHead};
  std::size_t head_out_reg_size_{2};
  std::size_t head_out_height_size_{1};
  std::size_t head_out_dim_size_{3};
  std::size_t head_out_rot_size_{2};
  std::size_t head_out_vel_size_{0};
  std::size_t num_proposals_{0};
  std::vector<float> post_center_range_;
  std::vector<float> distance_bin_upper_limits_;
  std::vector<float> detection_score_thresholds_;
  std::vector<float> yaw_norm_thresholds_;
  std::size_t bbox_grid_x_size_{};
  std::size_t bbox_grid_y_size_{};
  std::size_t det_grid_x_size_{};
  std::size_t det_grid_y_size_{};

  // Shared backbone and preprocessing.
  std::int64_t cloud_capacity_{};
  std::int64_t min_num_voxels_{};
  std::int64_t max_num_voxels_{};
  const std::int64_t num_point_feature_size_{4};  // x, y, z, intensity

  float min_x_range_{};
  float max_x_range_{};
  float min_y_range_{};
  float max_y_range_{};
  float min_z_range_{};
  float max_z_range_{};

  float voxel_x_size_{};
  float voxel_y_size_{};
  float voxel_z_size_{};

  std::int64_t grid_x_size_{};
  std::int64_t grid_y_size_{};
  std::int64_t grid_z_size_{};

  std::array<std::int64_t, 3> voxels_num_{};

  // PTv3 backbone output feature dimension.
  const std::int64_t backbone_feat_dim_{64};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PTV3_CONFIG_HPP_
