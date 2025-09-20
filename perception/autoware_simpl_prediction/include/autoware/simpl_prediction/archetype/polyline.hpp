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

#ifndef AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__POLYLINE_HPP_
#define AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__POLYLINE_HPP_

#include "autoware/simpl_prediction/archetype/agent.hpp"
#include "autoware/simpl_prediction/archetype/map.hpp"

#include <lanelet2_core/Forward.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace autoware::simpl_prediction::archetype
{
/**
 * @brief A container of the single polyline.
 */
class Polyline
{
public:
  using value_type = MapPoint;
  using size_type = std::vector<value_type>::size_type;
  using reference = std::vector<value_type>::reference;
  using const_reference = std::vector<value_type>::const_reference;
  using iterator = std::vector<value_type>::iterator;
  using const_iterator = std::vector<value_type>::const_iterator;

  Polyline() = default;

  /**
   * @brief Construct a new Polyline object and compute the center under the hood.
   *
   * @param waypoints Vector of waypoints.
   */
  explicit Polyline(lanelet::Id id, const std::vector<value_type> & waypoints)
  : id_(id), waypoints_(waypoints)
  {
    center_ = find_center();
  }

  lanelet::Id id() const { return id_; }

  /**
   * @brief Return the center of the polyline.
   */
  const value_type & center() const noexcept { return center_; }

  /**
   * @brief Return true if the polyline is empty.
   */
  bool empty() const noexcept { return waypoints_.empty(); }

  /**
   * @brief Return the number of elements in the polyline.
   */
  size_type size() const noexcept { return waypoints_.size(); }

  /**
   * @brief Return a read/write reference to the data at the specified element of the polyline.
   *
   * @param i Index of the element.
   */
  reference operator[](size_type i) { return waypoints_[i]; }

  /**
   * @brief Return a read only reference to the data at the specified element of the polyline.
   *
   * @param i Index of the element.
   */
  const_reference operator[](size_type i) const { return waypoints_[i]; }

  /**
   * @brief Return a read/write reference to the data at the specified element of the polyline.
   *
   * @param i Index of the element.
   */
  reference at(size_type i) { return waypoints_.at(i); }

  /**
   * @brief Return a read only reference to the data at the specified element of the polyline.
   *
   * @param i Index of the element.
   */
  const_reference at(size_type i) const { return waypoints_.at(i); }

  /**
   * @brief Return a read/write iterator that points to the first element of the polyline.
   */
  iterator begin() noexcept { return waypoints_.begin(); }

  /**
   * @brief Return a read only iterator that points to the first element of the polyline.
   */
  const_iterator begin() const noexcept { return waypoints_.begin(); }

  /**
   * @brief Return a read/write iterator that points to the last element of the polyline.
   */
  iterator end() noexcept { return waypoints_.end(); }

  /**
   * @brief Return a read only iterator that points to the last element of the polyline.
   */
  const_iterator end() const noexcept { return waypoints_.end(); }

  /**
   * @brief Return a read/write reference to the data at the first element of waypoints.
   */
  reference front() { return waypoints_.front(); }

  /**
   * @brief Return a read only reference to the data at the first element of waypoints.
   */
  const_reference front() const { return waypoints_.front(); }

  /**
   * @brief Return a read/write reference to the data at the last element of waypoints.
   */
  reference back() { return waypoints_.back(); }

  /**
   * @brief Return a read only reference to the data at the last element of waypoints.
   */
  const_reference back() const { return waypoints_.back(); }

  /**
   * @brief Return 2D distance to the state.
   *
   * @param state Agent state.
   */
  double distance_from(const AgentState & state) const noexcept
  {
    return center_.distance_from(state);
  }

  /**
   * @brief Return 2D distance to another point.
   *
   * @param other Another map point.
   */
  double distance_from(const MapPoint & other) const noexcept
  {
    return center_.distance_from(other);
  }

  /**
   * @brief Transform waypoints from map to input point coordinate frame.
   *
   * @param to_x X position.
   * @param to_y Y position.
   * @param to_yaw Yaw angle [rad].
   */
  Polyline transform(double to_x, double to_y, double to_yaw) const;

  /**
   * @brief Transfor waypoints from map to input state coordinate frame.
   *
   * @param to_state Agent state in the target coordinate frame.
   */
  Polyline transform(const AgentState & to_state) const;

private:
  /**
   * @brief Try to find the center of the polyline by linear interpolation.
   * @note Return zeros if the waypoints are empty.
   */
  value_type find_center() const;

  lanelet::Id id_;                     //! Polyline ID.
  std::vector<value_type> waypoints_;  //!< Points container.
  value_type center_;                  //!< Center of polyline.
};

/**
 * @brief Trim neighbor polylines that the distance of center from an agent is less than the
 * specified tolerance.
 *
 * @param polylines Vector of polylines.
 * @param state_from Agent state.
 * @param range_distance Tolerance value of distance from polyline to agent.
 */
std::vector<Polyline> trim_neighbors(
  const std::vector<Polyline> & polylines, const AgentState & state_from, double range_distance);
}  // namespace autoware::simpl_prediction::archetype
#endif  // AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__POLYLINE_HPP_
