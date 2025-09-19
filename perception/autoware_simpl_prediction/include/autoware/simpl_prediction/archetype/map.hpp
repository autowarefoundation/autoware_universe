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

#ifndef AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__MAP_HPP_
#define AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__MAP_HPP_

#include "autoware/simpl_prediction/archetype/agent.hpp"
#include "autoware/simpl_prediction/archetype/exception.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <tuple>
#include <vector>

namespace autoware::simpl_prediction::archetype
{
/**
 * @brief An enumerate to represent map labels.
 */
enum class MapLabel {
  ROADWAY = 0,
  BUS_LANE = 1,
  BIKE_LANE = 2,
  DASH_SOLID = 3,
  DASHED = 4,
  DOUBLE_DASH = 5,
  SOLID = 6,
  DOUBLE_SOLID = 7,
  SOLID_DASH = 8,
  CROSSWALK = 9,
  UNKNOWN = 10
};

/**
 * @brief A class to represent a single point in a map.
 */
struct MapPoint
{
  /**
   * @brief Construct a new MapPoint object with default values.
   */
  MapPoint() = default;

  /**
   * @brief Construct a new Lane Point object
   *
   * @param x Location x in map coordinate system.
   * @param y Location y in map coordinate system.
   * @param z Location z in map coordinate system.
   * @param label Label.
   */
  MapPoint(double x, double y, double z, MapLabel label) : x(x), y(y), z(z), label(label) {}

  static size_t num_attribute() { return 4; }

  /**
   * @brief Compute the 2D distance.
   */
  double distance() const noexcept { return std::hypot(x, y); }

  /**
   * @brief Compute the 2D distance from the specified another point.
   *
   * @param other Another point.
   */
  double distance_from(const MapPoint & other) const noexcept
  {
    return std::hypot(x - other.x, y - other.y);
  }

  /**
   * @brief Compute the 2D distance from the specified agent state.
   *
   * @param other Agent state.
   */
  double distance_from(const AgentState & state) const noexcept
  {
    return std::hypot(x - state.x, y - state.y);
  }

  /**
   * @brief Compute the difference of xy between myself and another one.
   *
   * @param other Another point.
   */
  std::tuple<double, double> diff(const MapPoint & other, bool normalize = true) const
  {
    const double vx = x - other.x;
    const double vy = y - other.y;
    if (normalize) {
      const double norm = std::clamp(std::hypot(vx, vy), 1e-6, 1e9);
      return std::make_tuple(vx / norm, vy / norm);
    } else {
      return {vx, vy};
    }
  }

  /**
   * @brief Perform linear interpolation by `(1 - t) * self + t * other`.
   *
   * @param other Other point.
   * @param t Weight of
   * @return MapPoint
   */
  MapPoint lerp(const MapPoint & other, double t) const
  {
    const auto ix = (1 - t) * x + t * other.x;
    const auto iy = (1 - t) * y + t * other.y;
    const auto iz = (1 - t) * z + t * other.z;

    return {ix, iy, iz, label};
  }

  double x{0.0};                      //!< Location x in map coordinate system.
  double y{0.0};                      //!< Location y in map coordinate system.
  double z{0.0};                      //!< Location z in map coordinate system.
  MapLabel label{MapLabel::UNKNOWN};  //!< Label.
};
}  // namespace autoware::simpl_prediction::archetype
#endif  // AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__MAP_HPP_
