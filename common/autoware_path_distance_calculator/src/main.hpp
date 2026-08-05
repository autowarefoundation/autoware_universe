// Copyright 2021 Tier IV, Inc.
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

#ifndef MAIN_HPP_
#define MAIN_HPP_

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>
#include <vector>

namespace autoware::path_distance_calculator
{

// Computes the remaining distance from a pose to the route goal, along the route's planned
// lanelet sequence. This is plain lanelet2 logic with no ROS node dependency, so it can be
// constructed and exercised on its own (e.g. from a unit test).
class RouteDistanceCalculator
{
public:
  void set_map(const autoware_map_msgs::msg::LaneletMapBin & msg);

  // Caches the lanelet sequence for the route: LaneletRoute::segments already contains the
  // lanelet IDs mission_planner planned the route through (accounting for lane exclusions, lane
  // changes, etc.), so it is looked up directly instead of re-deriving a path with our own
  // shortest-path search, which could disagree with the actual planned route.
  void set_route(const autoware_planning_msgs::msg::LaneletRoute & msg);

  // Returns nullopt if the map/route are not ready, or current_pose cannot be matched to the
  // cached lanelet sequence.
  std::optional<double> calculate_remaining_distance(
    const geometry_msgs::msg::Pose & current_pose) const;

  struct PathLane
  {
    lanelet::ConstLanelet lanelet;
    double length;
  };

private:
  std::optional<std::vector<PathLane>::const_iterator> find_current_lane(
    const geometry_msgs::msg::Pose & current_pose) const;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  bool is_map_ready_{false};

  geometry_msgs::msg::Pose goal_pose_;
  lanelet::ConstLanelet goal_lanelet_;
  std::vector<PathLane> shortest_path_lanes_;
  bool is_route_ready_{false};
};

}  // namespace autoware::path_distance_calculator

#endif  // MAIN_HPP_
