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

#include "autoware/behavior_path_bidirectional_traffic_module/utils.hpp"

#include <boost/geometry/algorithms/length.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Lanelet.h>

namespace autoware::behavior_path_planner
{

lanelet::Ids get_lane_ids(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory)
{
  lanelet::Ids lane_ids_in_trajectory;
  for (const auto & point : trajectory.get_internal_bases()) {
    auto lane_ids = (trajectory.lane_ids().compute(point));
    if (lane_ids.size() != 1) {
      continue;
    }
    if (lane_ids_in_trajectory.empty()) {
      lane_ids_in_trajectory.emplace_back(lane_ids.front());
    }
    if (lane_ids_in_trajectory.back() != lane_ids.front()) {
      lane_ids_in_trajectory.emplace_back(lane_ids.front());
    }
  }
  return lane_ids_in_trajectory;
}
double compute_length_of_lanelets(const lanelet::ConstLanelet & lanelet)
{
  auto centerline = lanelet::utils::to2D(lanelet.centerline());
  return static_cast<double>(boost::geometry::length(centerline));
}

}  // namespace autoware::behavior_path_planner
