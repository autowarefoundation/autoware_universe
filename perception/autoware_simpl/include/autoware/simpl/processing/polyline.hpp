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

#ifndef AUTOWARE__SIMPL__PROCESSING__POLYLINE_HPP_
#define AUTOWARE__SIMPL__PROCESSING__POLYLINE_HPP_

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/datatype.hpp"
#include "autoware/simpl/archetype/map.hpp"
#include "autoware/simpl/processing/geometry.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <vector>

namespace autoware::simpl::processing
{
/**
 * @brief Create a polylines by separating map points.
 *
 * @param map_points Vector of map points.
 * @param current_ego Current ego state.
 * @param max_num_point Max number of points contained in a single polyline.
 * @param break_distance Distance threshold to separate polylines.
 */
inline std::vector<archetype::MapPoints> create_polylines(
  const archetype::MapPoints & map_points, const archetype::AgentState & current_ego,
  size_t max_num_point, double break_distance)
{
  std::vector<archetype::MapPoints> polylines;
  size_t polyline_cnt = 0;
  size_t point_cnt = 0;
  for (size_t i = 0; i < map_points.size(); ++i) {
    const auto & current = transform2d(map_points.at(i), current_ego);
    if (i == 0) {
      archetype::MapPoints polyline{current};
      polylines.emplace_back(polyline);
      ++polyline_cnt;
      point_cnt = 1;
    } else {
      const auto & previous = polylines.back().back();
      if (previous.distance_from(current) > break_distance || point_cnt >= max_num_point) {
        // append new polyline
        archetype::MapPoints polyline{current};
        polylines.emplace_back(polyline);
        ++polyline_cnt;
        point_cnt = 1;
      } else {
        // append the point to the last polyline
        polylines.at(polyline_cnt - 1).emplace_back(current);
        ++point_cnt;
      }
    }
  }
  return polylines;
}

/**
 * @brief Find the center of the polyline by linear interpolation.
 *
 * @param map_points Vector of map points in a single polyline.
 */
inline archetype::MapPoint find_center(const archetype::MapPoints & map_points)
{
  if (map_points.size() < 2) {
    return map_points.empty() ? archetype::MapPoint() : map_points.front();
  }

  // Compute cumulative arc length
  std::vector<double> cumulative_length{0.0};
  for (size_t i = 1; i < map_points.size(); ++i) {
    const auto & p0 = map_points.at(i - 1);
    const auto & p1 = map_points.at(i);
    const auto & sum = cumulative_length.at(i - 1);
    cumulative_length.push_back(sum + p0.distance_from(p1));
  }

  // Find the segment where the midpoint length falls
  const auto middle = 0.5 * cumulative_length.back();
  const auto iter = std::lower_bound(cumulative_length.begin(), cumulative_length.end(), middle);
  if (iter == cumulative_length.end()) {
    return map_points.back();
  }

  // Linear interpolation
  const auto t = (middle - *iter) / (*(iter + 1) - *iter);
  const auto index = std::distance(cumulative_length.begin(), iter);
  const auto & first = map_points.at(index);
  const auto & second = map_points.at(index + 1);
  return first.interpolate(second, t);
}
}  // namespace autoware::simpl::processing

#endif  // AUTOWARE__SIMPL__PROCESSING__POLYLINE_HPP_
