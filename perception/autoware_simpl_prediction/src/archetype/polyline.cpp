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

#include "autoware/simpl_prediction/archetype/polyline.hpp"

#include "autoware/simpl_prediction/archetype/agent.hpp"
#include "autoware/simpl_prediction/archetype/map.hpp"

#include <numeric>
#include <vector>

namespace autoware::simpl_prediction::archetype
{
namespace
{
/**
 * @brief Transform map point.
 *
 * @param from Source map points.
 * @param to_x X coordinate of the target frame.
 * @param to_y Y coordinate of the target frame.
 * @param to_yaw Yaw angle of the target frame [rad].
 */
MapPoint transform_point(const archetype::MapPoint & from, double to_x, double to_y, double to_yaw)
{
  double vcos = std::cos(to_yaw);
  double vsin = std::sin(to_yaw);

  double x = (from.x - to_x) * vcos + (from.y - to_y) * vsin;
  double y = -(from.x - to_x) * vsin + (from.y - to_y) * vcos;

  return {x, y, from.z, from.label};
}
}  // namespace

Polyline Polyline::transform(double to_x, double to_y, double to_yaw) const
{
  std::vector<MapPoint> transformed;
  for (const auto & point : waypoints_) {
    transformed.emplace_back(transform_point(point, to_x, to_y, to_yaw));
  }
  return Polyline(id_, transformed);
}

Polyline Polyline::transform(const AgentState & to_state) const
{
  std::vector<MapPoint> transformed;
  for (const auto & point : waypoints_) {
    transformed.emplace_back(transform_point(point, to_state.x, to_state.y, to_state.yaw));
  }
  return Polyline(id_, transformed);
}

Polyline::value_type Polyline::find_center() const
{
  if (waypoints_.size() < 2) {
    return waypoints_.empty() ? archetype::MapPoint() : waypoints_.front();
  }

  // 1. Distances between consecutive points
  std::vector<double> dist(waypoints_.size() - 1);
  std::transform(
    waypoints_.begin(), std::prev(waypoints_.end()), std::next(waypoints_.begin()), dist.begin(),
    [](auto const & p1, auto const & p2) { return p1.distance_from(p2); });

  // 2. Cumulative distances
  std::vector<double> cumulative_length(waypoints_.size());
  cumulative_length[0] = 0.0;
  std::partial_sum(dist.begin(), dist.end(), std::next(cumulative_length.begin()));

  // 3. Midpoint distance
  const double total_length = cumulative_length.back();
  const double middle = 0.5 * total_length;

  // 4. Locate segment with lower_bound
  auto it = std::lower_bound(cumulative_length.begin(), cumulative_length.end(), middle);
  if (it == cumulative_length.end()) {
    return waypoints_.back();
  }

  // 5. Adjust index (step back if not at the very beginning)
  std::size_t segment_index =
    static_cast<std::size_t>(std::distance(cumulative_length.begin(), it));
  if (segment_index > 0) {
    --segment_index;
  }

  // 6. Linear interpolation
  const double segment_start = cumulative_length[segment_index];
  const double segment_end = cumulative_length[segment_index + 1];

  const double denom = segment_end - segment_start;
  const double t = denom > 1e-6 ? (middle - segment_start) / denom : 0.0;

  const auto & p0 = waypoints_[segment_index];
  const auto & p1 = waypoints_[segment_index + 1];
  return p0.lerp(p1, t);
}

std::vector<Polyline> trim_neighbors(
  const std::vector<Polyline> & polylines, const AgentState & state_from, double distance_tolerance)
{
  std::vector<Polyline> output;
  for (const auto & polyline : polylines) {
    if (polyline.distance_from(state_from) > distance_tolerance) {
      continue;
    }
    output.emplace_back(polyline);
  }
  return output;
}
}  // namespace autoware::simpl_prediction::archetype
