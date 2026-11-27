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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__UTILITY__AZIMUTH_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__UTILITY__AZIMUTH_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace autoware::pointcloud_preprocessor::utils
{
/**
 * @brief Angular distance in [0, pi] between two azimuths, in radians.
 *
 * It does not depend on the scan direction: azimuths that increase along a ring (nebula drivers)
 * and azimuths that decrease along a ring (e.g. atan2(y, x) of a clockwise-scanning sensor) give
 * the same result. It also handles the wrap-around anywhere on the circle, so both [0, 2pi) and
 * [-pi, pi) are accepted, as long as both azimuths are in the same 2pi-wide interval.
 *
 * It compiles to abs + min without branches, so it is as fast as the signed difference with a
 * conditional 2pi offset that it replaces.
 */
inline float azimuth_distance(const float from, const float to)
{
  constexpr float two_pi = static_cast<float>(2.0 * M_PI);
  const float diff = std::fabs(to - from);
  return std::min(diff, two_pi - diff);
}

/**
 * @brief Signed azimuth step from `from` to `to` in [-pi, pi], in radians.
 *
 * Positive when the azimuth increases (counterclockwise), negative when it decreases. The
 * wrap-around is handled anywhere on the circle, like azimuth_distance(), whose value is
 * std::fabs(azimuth_step(from, to)). It stays in float with two branches instead of using
 * autoware_utils_math::normalize_radian(), which works in double with fmod.
 */
inline float azimuth_step(const float from, const float to)
{
  constexpr float two_pi = static_cast<float>(2.0 * M_PI);
  constexpr float pi = static_cast<float>(M_PI);
  float step = to - from;
  if (step > pi) {
    step -= two_pi;
  } else if (step < -pi) {
    step += two_pi;
  }
  return step;
}

/**
 * @brief Counts the directions of the azimuth steps along one ring, to tell whether the points
 * are in scan order.
 *
 * In scan order the azimuth moves in one direction only, either one. Steps of 0 (for example the
 * returns of a multi-return sensor at the same azimuth) are not counted. A ring is out of order
 * when more than max_minority_ratio of the counted steps go against the majority direction. A few
 * backward steps, such as jitter or the end of a scan, are tolerated.
 */
class AzimuthOrderCounter
{
public:
  /// largest tolerated share of steps against the majority direction
  static constexpr float max_minority_ratio = 0.05F;
  /// rings with fewer counted steps are always considered in order
  static constexpr std::size_t min_steps = 10;
  /// add_sampled() counts one step out of sample_stride
  static constexpr std::size_t sample_stride = 8;

  void add(const float step)
  {
    forward_ += static_cast<std::size_t>(step > 0.0F);
    backward_ += static_cast<std::size_t>(step < 0.0F);
  }

  /**
   * @brief Counts the step from `from` to `to` when step_index is a multiple of sample_stride.
   *
   * For callers in a hot loop: one step out of 8 is enough to detect shuffled rings.
   */
  void add_sampled(const std::size_t step_index, const float from, const float to)
  {
    if (step_index % sample_stride == 0) {
      add(azimuth_step(from, to));
    }
  }

  /// Whether the counted steps are in scan order.
  [[nodiscard]] bool is_ordered() const
  {
    const std::size_t total = forward_ + backward_;
    if (total < min_steps) {
      return true;
    }
    const std::size_t minority = std::min(forward_, backward_);
    return static_cast<float>(minority) <= max_minority_ratio * static_cast<float>(total);
  }

  [[nodiscard]] std::size_t forward() const { return forward_; }
  [[nodiscard]] std::size_t backward() const { return backward_; }

private:
  std::size_t forward_{0};
  std::size_t backward_{0};
};
}  // namespace autoware::pointcloud_preprocessor::utils

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__UTILITY__AZIMUTH_HPP_
