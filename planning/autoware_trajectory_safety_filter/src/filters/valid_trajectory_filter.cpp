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

#include "autoware/trajectory_safety_filter/filters/valid_trajectory_filter.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <algorithm>
#include <any>
#include <cmath>
#include <string>
#include <unordered_map>

namespace autoware::trajectory_safety_filter::plugin
{

void ValidTrajectory::initialize(const std::string & name)
{
  (void)name;  // Suppress unused parameter warning
  // Simple initialization with name only
  // Parameters will be set via set_parameters()
}

void ValidTrajectory::set_parameters(const std::unordered_map<std::string, std::any> & params)
{
  auto get_value = [&params](const std::string & key, auto & value) {
    auto it = params.find(key);
    if (it != params.end()) {
      try {
        value = std::any_cast<std::decay_t<decltype(value)>>(it->second);
      } catch (const std::bad_any_cast &) {
        // Keep default value if cast fails
      }
    }
  };

  get_value("max_distance_from_trajectory", params_.max_distance_from_trajectory);
  get_value("check_finite_values", params_.check_finite_values);
  get_value("check_trajectory_length", params_.check_trajectory_length);
  get_value("min_trajectory_length", params_.min_trajectory_length);
}

bool ValidTrajectory::filter_trajectory(TrajectoryPoints & points, const FilterContext & context)
{
  // Check trajectory length
  if (params_.check_trajectory_length && points.size() < params_.min_trajectory_length) {
    // Log warning (in production, use external logger)
    // For testing, this can be mocked or ignored
    return false;
  }

  // Check finite values
  if (params_.check_finite_values && !check_trajectory_validity(points)) {
    // Log warning: Trajectory contains non-finite values
    return false;
  }

  // Check if trajectory is off-track
  if (context.current_pose) {
    const auto & ego_position = context.current_pose->pose.position;
    if (check_off_track(points, ego_position)) {
      // Log warning: Trajectory is off-track from current position
      return false;
    }
  }

  return true;
}

bool ValidTrajectory::check_trajectory_validity(const TrajectoryPoints & points) const
{
  return std::all_of(
    points.begin(), points.end(), [this](const auto & point) { return check_finite(point); });
}

bool ValidTrajectory::check_finite(const TrajectoryPoint & point) const
{
  const auto & p = point.pose.position;
  const auto & o = point.pose.orientation;

  using std::isfinite;
  const bool p_result = isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
  const bool quat_result = isfinite(o.x) && isfinite(o.y) && isfinite(o.z) && isfinite(o.w);
  const bool v_result = isfinite(point.longitudinal_velocity_mps);
  const bool w_result = isfinite(point.heading_rate_rps);
  const bool a_result = isfinite(point.acceleration_mps2);

  return p_result && quat_result && v_result && w_result && a_result;
}

bool ValidTrajectory::check_off_track(
  const TrajectoryPoints & points, const geometry_msgs::msg::Point & ego_position) const
{
  const auto idx = autoware::motion_utils::findNearestIndex(points, ego_position);
  const auto & target_position = points.at(idx).pose.position;

  const double distance_sq =
    autoware::universe_utils::calcSquaredDistance2d(ego_position, target_position);
  const double threshold_sq =
    params_.max_distance_from_trajectory * params_.max_distance_from_trajectory;

  return distance_sq > threshold_sq;
}

}  // namespace autoware::trajectory_safety_filter::plugin
