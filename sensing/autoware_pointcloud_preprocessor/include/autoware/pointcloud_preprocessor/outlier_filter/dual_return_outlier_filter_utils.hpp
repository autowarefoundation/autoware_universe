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

  static constexpr float two_pi = static_cast<float>(2.0 * M_PI);

  /** @brief Azimuth wrapped into [0, 2pi). */
  static float wrap(const float azimuth)
  {
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

/** @brief Open interval (min, max). */
struct OpenRange
{
  float min;
  float max;

  bool contains(const float value) const { return value > min && value < max; }
};

/** @brief Box in which a point is inside when each coordinate is strictly inside its range. */
struct XyzRoi
{
  OpenRange x;
  OpenRange y;
  OpenRange z;

  /** @tparam PointT a type with float members `x`, `y` and `z` */
  template <typename PointT>
  bool contains(const PointT & p) const
  {
    return x.contains(p.x) && y.contains(p.y) && z.contains(p.z);
  }
};

/** @brief Parameters of classify_weak_first_ring(). */
struct WeakFirstParams
{
  /// distance_ratio of find_segment_points()
  double distance_ratio;
  /// max_azimuth_diff of find_segment_points(), in radians
  float max_azimuth_diff;
  /// A bin with at least this many isolated points in the ROI is noisy
  int noise_threshold;
};

/** @brief Classification of the weak first returns of one ring. */
struct WeakFirstRingResult
{
  /// For each point of the ring, whether it is noise
  std::vector<bool> is_noise;
  /// For each bin, the number of noise points in the ROI
  std::vector<int> noise_frequency;
};

/**
 * @brief Classifies the weak first returns of one ring.
 *
 * A point that is not in a segment (see find_segment_points()) is noise. A bin with at least
 * noise_threshold such isolated points in the ROI is noisy, and the segment points in it are noise
 * as well. The decision uses the counts of the isolated points only, so it does not depend on the
 * order of the points. noise_frequency counts every noise point in the ROI, isolated or not.
 *
 * @tparam PointT a type with float members `distance` and `azimuth`
 * @tparam RoiPredicate a callable `bool(const PointT &)`, true for a point in the ROI
 */
template <typename PointT, typename RoiPredicate>
WeakFirstRingResult classify_weak_first_ring(
  const std::vector<PointT> & ring, const AzimuthBins & bins, const WeakFirstParams & params,
  const RoiPredicate & is_in_roi)
{
  WeakFirstRingResult result{
    find_segment_points(ring, params.distance_ratio, params.max_azimuth_diff),
    std::vector<int>(bins.size(), 0)};
  // find_segment_points() gives in_segment, so noise is its negation
  result.is_noise.flip();

  const auto count_if_in_roi = [&](const PointT & p) {
    const auto bin = bins.bin(p.azimuth);
    if (bin && is_in_roi(p)) {
      ++result.noise_frequency[*bin];
    }
  };

  for (std::size_t i = 0; i < ring.size(); ++i) {
    if (result.is_noise[i]) {
      count_if_in_roi(ring[i]);
    }
  }
  const std::vector<int> isolated_frequency = result.noise_frequency;

  for (std::size_t i = 0; i < ring.size(); ++i) {
    if (result.is_noise[i]) {
      continue;
    }
    const auto bin = bins.bin(ring[i].azimuth);
    if (!bin || isolated_frequency[*bin] < params.noise_threshold) {
      continue;
    }
    result.is_noise[i] = true;
    count_if_in_roi(ring[i]);
  }
  return result;
}
}  // namespace autoware::pointcloud_preprocessor::dual_return_outlier_filter

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__DUAL_RETURN_OUTLIER_FILTER_UTILS_HPP_
