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
  constexpr float two_pi = 6.28318530717958647692F;
  const float diff = std::fabs(to - from);
  return std::min(diff, two_pi - diff);
}
}  // namespace autoware::pointcloud_preprocessor::utils

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__UTILITY__AZIMUTH_HPP_
