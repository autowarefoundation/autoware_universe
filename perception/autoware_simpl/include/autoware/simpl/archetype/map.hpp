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

#ifndef AUTOWARE__SIMPL__ARCHETYPE__MAP_HPP_
#define AUTOWARE__SIMPL__ARCHETYPE__MAP_HPP_

#include "autoware/simpl/archetype/exception.hpp"

#include <cmath>
#include <cstddef>
#include <sstream>
#include <tuple>
#include <vector>

namespace autoware::simpl::archetype
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
class MapPoint
{
public:
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
   * @param dx X difference between previous point.
   * @param dy Y difference between previous point.
   * @param dz Z difference between previous point.
   * @param label Label.
   */
  MapPoint(double x, double y, double z, double dx, double dy, double dz, MapLabel label)
  : x(x), y(y), z(z), dx(dx), dy(dy), dz(dz), label(label)
  {
  }

  static size_t num_attribute() { return 7; }

  /**
   * @brief Compute the distance.
   */
  double distance() const noexcept { return std::hypot(x, y, z); }

  /**
   * @brief Compute the distance from the specified another point.
   *
   * @param other Another point.
   */
  double distance_from(const MapPoint & other) const noexcept
  {
    return std::hypot(x - other.x, y - other.y, z - other.z);
  }

  /**
   * @brief Compute the difference of xyz between myself and another one.
   *
   * @param other Another point.
   */
  std::tuple<double, double, double> diff(const MapPoint & other) const
  {
    return {x - other.x, y - other.y, z - other.z};
  }

  /**
   * @brief Execute linear interpolation by `(1 - t) * self + t * other`.
   *
   * Note that dx/dy/dz results are always `0.0`.
   *
   * @param other Other point.
   * @param t Weight of
   * @return MapPoint
   */
  MapPoint interpolate(const MapPoint & other, double t) const
  {
    const auto ix = (1 - t) * x + t * other.x;
    const auto iy = (1 - t) * y + t * other.y;
    const auto iz = (1 - t) * z + t * other.z;

    return {ix, iy, iz, 0.0, 0.0, 0.0, label};
  }

  double x{0.0};                      //!< Location x in map coordinate system.
  double y{0.0};                      //!< Location y in map coordinate system.
  double z{0.0};                      //!< Location z in map coordinate system.
  double dx{0.0};                     //!< X difference between previous point.
  double dy{0.0};                     //!< Y difference between previous point.
  double dz{0.0};                     //!< Z difference between previous point.
  MapLabel label{MapLabel::UNKNOWN};  //!< Label.
};

/**
 * @brief A class to represent map tensor data.
 */
class MapTensor
{
public:
  using size_type = std::vector<float>::size_type;

  /**
   * @brief Construct a new MapTensor object.
   *
   * @param tensor 1D map tensor data in the shape of (K*P*Dm).
   * @param num_polyline Number of polylines (K).
   * @param num_point Number of points contained in a single polyline (P).
   * @param num_attribute Number of attributes (Dm).
   */
  MapTensor(
    const std::vector<float> & tensor, size_t num_polyline, size_t num_point, size_t num_attribute)
  : num_polyline(num_polyline), num_point(num_point), num_attribute(num_attribute), tensor_(tensor)
  {
    if (tensor_.size() != num_polyline * (num_point - 1) * num_attribute) {
      std::ostringstream msg;
      msg << "Invalid size of map tensor: " << tensor_.size()
          << " != " << num_polyline * (num_point - 1) * num_attribute;
      throw SimplException(SimplError_t::InvalidValue, msg.str());
    }
  }

  /**
   * @brief Return the pointer to tensor data.
   */
  const float * data() const noexcept { return tensor_.data(); }

  /**
   * @brief Return the size of tensor elements, where K*P*D.
   */
  size_type size() const noexcept { return tensor_.size(); }

  const size_t num_polyline;   //!< Number of polylines (K).
  const size_t num_point;      //!< Number of points contained in a single polyline (P).
  const size_t num_attribute;  //!< Number of attributes (Dm).

private:
  std::vector<float> tensor_;  //!< Map tensor data.
};
}  // namespace autoware::simpl::archetype
#endif  // AUTOWARE__SIMPL__ARCHETYPE__MAP_HPP_
