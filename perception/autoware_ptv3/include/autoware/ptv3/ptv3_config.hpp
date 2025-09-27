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
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ptv3
{

class PTv3Config
{
public:
  PTv3Config(
    const std::string & plugins_path, const std::int64_t cloud_capacity,
    const std::vector<std::int64_t> & voxels_num, const std::vector<float> & point_cloud_range,
    const std::vector<float> & voxel_size, const std::vector<std::int64_t> & colors_red,
    const std::vector<std::int64_t> & colors_green, const std::vector<std::int64_t> & colors_blue,
    const std::vector<std::string> & class_names, const float ground_prob_threshold)
  {
    plugins_path_ = plugins_path;

    cloud_capacity_ = cloud_capacity;

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

    class_names_ = class_names;

    if (
      colors_red.size() != class_names_.size() || colors_green.size() != class_names_.size() ||
      colors_blue.size() != class_names_.size()) {
      throw std::runtime_error(
        "The size of colors_red, colors_green, and colors_blue must be the same as class_names");
    }

    for (std::size_t i = 0; i < class_names_.size(); ++i) {
      auto r = static_cast<std::uint8_t>(colors_red[i]);
      auto g = static_cast<std::uint8_t>(colors_green[i]);
      auto b = static_cast<std::uint8_t>(colors_blue[i]);
      std::uint32_t rgb = (r << 16) | (g << 8) | b;
      float rgb_float;
      memcpy(&rgb_float, &rgb, sizeof(rgb_float));
      colors_rgb_.push_back(rgb_float);

      std::string class_name = class_names_[i];
      std::transform(class_name.begin(), class_name.end(), class_name.begin(), [](unsigned char c) {
        return std::tolower(c);
      });

      if (class_name == "ground") {
        ground_label_ = static_cast<std::int32_t>(i);
      }
    }

    if (ground_label_ == -1) {
      throw std::runtime_error("Ground label not found in class names");
    }

    ground_prob_threshold_ = ground_prob_threshold;
  }

  // CUDA parameters
  const std::uint32_t threads_per_block_{256};  // threads number for a block

  // TensorRT parameters
  std::string plugins_path_{};

  // Preprocess parameters
  bool use_64bit_hash_{};
  std::int32_t serialization_depth_{};

  ///// NETWORK PARAMETERS /////

  // Head parameters
  std::vector<std::string> class_names_{};
  std::vector<float> colors_rgb_{};
  float ground_prob_threshold_{};
  std::int32_t ground_label_{-1};

  // Common network parameters
  std::int64_t cloud_capacity_{};
  std::int64_t min_num_voxels_{};
  std::int64_t max_num_voxels_{};
  const std::int64_t num_point_feature_size_{4};  // x, y, z, intensity

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
