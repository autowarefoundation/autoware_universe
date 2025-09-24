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

#include "autoware/trajectory_safety_filter/filters/out_of_lane_filter.hpp"

#include <rclcpp/duration.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>

#include <algorithm>
#include <any>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_safety_filter::plugin
{

void OutOfLaneFilter::set_parameters(const std::unordered_map<std::string, std::any> & params)
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

  get_value("max_check_time", params_.max_check_time);
  get_value("min_value", params_.min_value);
}

bool OutOfLaneFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  // Check required context data
  if (!context.lanelet_map || !context.odometry) {
    return true;
  }

  for (const auto & point : traj_points) {
    if (rclcpp::Duration(point.time_from_start).seconds() > params_.max_check_time) {
      break;
    }
    const auto nearst_lanelets = lanelet::geometry::findWithin2d(
      context.lanelet_map->laneletLayer,
      lanelet::BasicPoint2d(point.pose.position.x, point.pose.position.y), 0.0);
    if (nearst_lanelets.empty()) {
      return false;
    }
  }

  return true;
}
}  // namespace autoware::trajectory_safety_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_safety_filter::plugin::OutOfLaneFilter,
  autoware::trajectory_safety_filter::plugin::SafetyFilterInterface)
