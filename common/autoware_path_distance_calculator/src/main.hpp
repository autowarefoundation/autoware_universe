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
#include <lanelet2_core/utility/Optional.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <optional>
#include <vector>

namespace autoware::path_distance_calculator
{

// Computes the remaining distance from a pose to the route goal, along the shortest path (in
// lanelets) between them. This is plain lanelet2 logic with no ROS node dependency, so it can be
// constructed and exercised on its own (e.g. from a unit test).
class RouteDistanceCalculator
{
public:
  void set_map(const autoware_map_msgs::msg::LaneletMapBin & msg);

  // Resolves and caches the shortest path from current_pose to the route goal. Remaining
  // distance queries below are answered against this cached path, not recomputed from scratch,
  // matching how autoware_remaining_distance_time_calculator handles route updates.
  void set_route(
    const autoware_planning_msgs::msg::LaneletRoute & msg,
    const geometry_msgs::msg::Pose & current_pose);

  // Returns nullopt if the map/route are not ready, or current_pose cannot be matched to the
  // cached shortest path.
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
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::ConstLanelets road_lanes_;
  bool is_map_ready_{false};

  geometry_msgs::msg::Pose goal_pose_;
  lanelet::ConstLanelet goal_lanelet_;
  std::vector<PathLane> shortest_path_lanes_;
  bool is_route_ready_{false};
};

}  // namespace autoware::path_distance_calculator

#endif  // MAIN_HPP_
