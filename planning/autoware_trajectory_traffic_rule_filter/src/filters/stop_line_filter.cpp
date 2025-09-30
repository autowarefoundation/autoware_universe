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

#include "autoware/trajectory_traffic_rule_filter/filters/stop_line_filter.hpp"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <algorithm>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter::plugin
{

StopLineFilter::StopLineFilter() : TrafficRuleFilterInterface("StopLineFilter")
{
}

lanelet::ConstLanelets StopLineFilter::get_lanelets_from_trajectory(
  const TrajectoryPoints & trajectory_points) const
{
  lanelet::ConstLanelets lanes;

  if (!lanelet_map_ || trajectory_points.empty()) {
    return lanes;
  }

  const lanelet::ConstLanelets all_lanelets(
    lanelet_map_->laneletLayer.begin(), lanelet_map_->laneletLayer.end());

  for (const auto & point : trajectory_points) {
    const auto p = lanelet::BasicPoint2d(point.pose.position.x, point.pose.position.y);

    for (const auto & lanelet : all_lanelets) {
      if (lanelet::geometry::inside(lanelet, p)) {
        if (std::find(lanes.begin(), lanes.end(), lanelet) == lanes.end()) {
          lanes.push_back(lanelet);
        }
        break;
      }
    }
  }

  return lanes;
}

bool StopLineFilter::is_feasible(const TrajectoryPoints & trajectory_points)
{
  if (!lanelet_map_ || trajectory_points.empty()) {
    return true;
  }

  const auto lanes = get_lanelets_from_trajectory(trajectory_points);

  if (lanes.size() < 2) {
    return true;
  }

  for (size_t i = 0; i < lanes.size() - 1; i++) {
    for (const auto & reg_elem : lanes[i].regulatoryElementsAs<lanelet::TrafficSign>()) {
      if (reg_elem->type() != "stop_sign") continue;
      if (lanes[i].id() != lanes[i + 1].id()) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace autoware::trajectory_traffic_rule_filter::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_traffic_rule_filter::plugin::StopLineFilter,
  autoware::trajectory_traffic_rule_filter::plugin::TrafficRuleFilterInterface)
