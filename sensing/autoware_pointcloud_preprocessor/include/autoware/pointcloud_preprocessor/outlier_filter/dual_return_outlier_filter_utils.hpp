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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__DUAL_RETURN_OUTLIER_FILTER_UTILS_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__DUAL_RETURN_OUTLIER_FILTER_UTILS_HPP_

#include "autoware/pointcloud_preprocessor/utility/azimuth.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::pointcloud_preprocessor::dual_return_outlier_filter
{
/**
 * @brief Horizontal bins of the visibility histogram, over [min_azimuth, max_azimuth) in radians.
 *
 * The bin of a point is computed from its own azimuth, so the result does not depend on the order
 * of the points, nor on the scan direction. Azimuths are wrapped into [0, 2pi) first, so input in
 * [-pi, pi) is accepted too.
 */
class AzimuthBins
{
public:
  AzimuthBins(const float min_azimuth, const float max_azimuth, const std::size_t num_bins)
  : min_azimuth_(min_azimuth),
    max_azimuth_(max_azimuth),
    num_bins_(num_bins),
    inverse_resolution_(
      max_azimuth > min_azimuth && num_bins > 0
        ? static_cast<float>(num_bins) / (max_azimuth - min_azimuth)
        : 0.0F)
  {
  }

  /** @brief Azimuth wrapped into [0, 2pi). */
  static float wrap(const float azimuth)
  {
    constexpr float two_pi = static_cast<float>(2.0 * M_PI);
    float wrapped = azimuth < 0.0F ? azimuth + two_pi : azimuth;
    wrapped = wrapped >= two_pi ? wrapped - two_pi : wrapped;
    return wrapped;
  }

  /** @brief Whether the azimuth is in [min_azimuth, max_azimuth). */
  bool contains(const float azimuth) const
  {
    const float wrapped = wrap(azimuth);
    return wrapped >= min_azimuth_ && wrapped < max_azimuth_;
  }

  /** @brief Bin index of the azimuth, or nullopt if it is outside [min_azimuth, max_azimuth). */
  std::optional<std::size_t> bin(const float azimuth) const
  {
    if (inverse_resolution_ <= 0.0F || !contains(azimuth)) {
      return std::nullopt;
    }
    const auto index =
      static_cast<std::size_t>((wrap(azimuth) - min_azimuth_) * inverse_resolution_);
    // Float rounding can give num_bins for an azimuth just below max_azimuth
    return std::min(index, num_bins_ - 1);
  }

  std::size_t size() const { return num_bins_; }

private:
  float min_azimuth_;
  float max_azimuth_;
  std::size_t num_bins_;
  float inverse_resolution_;
};

/**
 * @brief For each point of a ring, whether it belongs to a segment of neighbouring points.
 *
 * Two consecutive points are neighbours when the larger distance is below the smaller one times
 * distance_ratio and their azimuth step is below max_azimuth_diff (radians, either direction). A
 * point is in a segment when it is a neighbour of the point before it or of the point after it.
 * Every point is classified, including the first and the last one of the ring.
 *
 * @tparam PointT a type with float members `distance` and `azimuth`
 */
template <typename PointT>
std::vector<bool> find_segment_points(
  const std::vector<PointT> & ring, const double distance_ratio, const float max_azimuth_diff)
{
  std::vector<bool> in_segment(ring.size(), false);
  for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
    const auto & current = ring[i];
    const auto & next = ring[i + 1];
    const float min_dist = std::min(current.distance, next.distance);
    const float max_dist = std::max(current.distance, next.distance);
    const float azimuth_diff = utils::azimuth_distance(current.azimuth, next.azimuth);
    if (max_dist < min_dist * distance_ratio && azimuth_diff < max_azimuth_diff) {
      in_segment[i] = true;
      in_segment[i + 1] = true;
    }
  }
  return in_segment;
}
}  // namespace autoware::pointcloud_preprocessor::dual_return_outlier_filter

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__DUAL_RETURN_OUTLIER_FILTER_UTILS_HPP_
