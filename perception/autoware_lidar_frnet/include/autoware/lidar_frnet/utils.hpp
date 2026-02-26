// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__LIDAR_FRNET__UTILS_HPP_
#define AUTOWARE__LIDAR_FRNET__UTILS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::lidar_frnet::utils
{
constexpr uint32_t kernel_1d_size = 256;
constexpr uint32_t kernel_2d_size = 16;

struct ActiveComm
{
  ActiveComm(const bool is_seg_active, const bool is_viz_active, const bool is_filtered_active)
  : seg(is_seg_active), viz(is_viz_active), filtered(is_filtered_active)
  {
  }
  bool seg;
  bool viz;
  bool filtered;

  explicit operator bool() const { return seg || viz || filtered; }
};

struct Dims2d
{
  Dims2d() = default;
  Dims2d(const int64_t width, const int64_t height)
  : w(static_cast<uint32_t>(width)), h(static_cast<uint32_t>(height))
  {
    if (width <= 0 || height <= 0) {
      throw std::runtime_error("Width and height must be positive.");
    }
  }
  uint32_t w;
  uint32_t h;
};

struct FieldOfView
{
  FieldOfView() = default;
  FieldOfView(const double fov_up_deg, const double fov_down_deg)
  : up(static_cast<float>(M_PI * fov_up_deg / 180.0)),
    down(static_cast<float>(M_PI * fov_down_deg / 180.0)),
    total(static_cast<float>(M_PI * std::fabs(fov_up_deg - fov_down_deg) / 180.0))
  {
  }
  float up;
  float down;
  float total;
};

struct PostprocessingParams
{
  PostprocessingParams(
    const double filter_class_confidence_threshold, const std::vector<std::string> & filter_classes,
    const std::array<float, 6> & crop_box_bounds, const std::vector<std::string> & class_names,
    const std::vector<int64_t> & palette)
  : filter_class_confidence_threshold(static_cast<float>(filter_class_confidence_threshold)),
    palette(make_palette(class_names, palette)),
    filter_class_indices(make_filter_class_indices(class_names, filter_classes)),
    crop_box_bounds(crop_box_bounds)
  {
  }

  static std::vector<float> make_palette(
    const std::vector<std::string> & class_names, const std::vector<int64_t> & palette)
  {
    if (palette.size() % 3 != 0) {
      throw std::runtime_error("Palette size must be a multiple of 3.");
    }
    if (palette.size() != class_names.size() * 3) {
      throw std::runtime_error("Palette size does not match class names size.");
    }
    std::vector<float> colors;
    for (size_t i = 0; i < palette.size(); i += 3) {
      const auto & r = palette[i];
      const auto & g = palette[i + 1];
      const auto & b = palette[i + 2];
      if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
        throw std::runtime_error("Color values must be within 0-255 range.");
      }
      const uint32_t color = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) |
                             (static_cast<uint32_t>(b));
      float result = 0.0f;
      memcpy(&result, &color, sizeof(result));
      colors.emplace_back(result);
    }
    return colors;
  }

  static std::vector<uint32_t> make_filter_class_indices(
    const std::vector<std::string> & class_names, const std::vector<std::string> & filter_classes)
  {
    std::vector<uint32_t> indices;
    for (const auto & filter_class : filter_classes) {
      auto it = std::find(class_names.begin(), class_names.end(), filter_class);
      if (it == class_names.end()) {
        throw std::runtime_error("Filter class '" + filter_class + "' not found in class names.");
      }
      indices.push_back(static_cast<uint32_t>(std::distance(class_names.begin(), it)));
    }
    return indices;
  }
  const float filter_class_confidence_threshold;
  const std::vector<float> palette;
  const std::vector<uint32_t> filter_class_indices;
  const std::array<float, 6> crop_box_bounds;  // [min_x, min_y, min_z, max_x, max_y, max_z]
};

struct Profile
{
  explicit Profile(const std::vector<int64_t> & profile)
  : min(profile.at(0)), opt(profile.at(1)), max(profile.at(2))
  {
  }
  const int64_t min;
  const int64_t opt;
  const int64_t max;
};

struct NetworkParams
{
  NetworkParams(
    const std::vector<std::string> & class_names, const std::vector<int64_t> & num_points,
    const std::vector<int64_t> & num_unique_coors, const double fov_up_deg,
    const double fov_down_deg, const int64_t frustum_width, const int64_t frustum_height,
    const int64_t interpolation_width, const int64_t interpolation_height,
    bool crop_box_enabled = false, std::array<float, 6> crop_box_bounds = {})
  : class_names(class_names),
    num_points_profile(num_points),
    num_unique_coors_profile(num_unique_coors),
    num_classes(class_names.size()),
    fov(fov_up_deg, fov_down_deg),
    frustum(frustum_width, frustum_height),
    interpolation(interpolation_width, interpolation_height),
    crop_box_enabled(crop_box_enabled),
    crop_box_bounds(crop_box_bounds)
  {
  }
  const std::vector<std::string> class_names;
  const Profile num_points_profile;
  const Profile num_unique_coors_profile;
  const uint32_t num_classes;
  const FieldOfView fov;
  const Dims2d frustum;
  const Dims2d interpolation;
  const bool crop_box_enabled;
  const std::array<float, 6> crop_box_bounds;  // [min_x, min_y, min_z, max_x, max_y, max_z]
};

struct DiagnosticParams
{
  DiagnosticParams() = default;
  DiagnosticParams(
    const double max_allowed_processing_time_ms, const double max_acceptable_consecutive_delay_ms,
    const double validation_callback_interval_ms)
  : max_allowed_processing_time_ms(max_allowed_processing_time_ms),
    max_acceptable_consecutive_delay_ms(max_acceptable_consecutive_delay_ms),
    validation_callback_interval_ms(validation_callback_interval_ms)
  {
  }
  double max_allowed_processing_time_ms;
  double max_acceptable_consecutive_delay_ms;
  double validation_callback_interval_ms;
};

template <typename T1, typename T2>
uint32_t divup(const T1 a, const T2 b)
{
  if (a <= 0) {
    throw std::runtime_error("A dividend of divup isn't positive.");
  }
  if (b <= 0) {
    throw std::runtime_error("A divisor of divup isn't positive.");
  }

  return (a + b - 1) / b;
}

}  // namespace autoware::lidar_frnet::utils

#endif  // AUTOWARE__LIDAR_FRNET__UTILS_HPP_
