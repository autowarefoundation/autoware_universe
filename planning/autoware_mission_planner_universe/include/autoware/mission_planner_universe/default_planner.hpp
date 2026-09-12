// Copyright 2019 Autoware Foundation
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

#ifndef AUTOWARE__MISSION_PLANNER_UNIVERSE__DEFAULT_PLANNER_HPP_
#define AUTOWARE__MISSION_PLANNER_UNIVERSE__DEFAULT_PLANNER_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::mission_planner_universe::lanelet2
{
struct DefaultPlannerParameters
{
  double goal_angle_threshold_deg;
  bool enable_correct_goal_pose;
  bool consider_no_drivable_lanes;
  bool check_footprint_inside_lanes;
  bool allow_area;
};

class DefaultPlanner
{
public:
  using RoutePoints = std::vector<geometry_msgs::msg::Pose>;
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  // goal_footprint is only set when is_goal_valid() actually computed the footprint (some early
  // return paths, e.g. the shoulder-lanelet check, never reach that computation).
  struct PlanResult
  {
    LaneletRoute route;
    std::optional<autoware_utils::LinearRing2d> goal_footprint;
    std::optional<std::string> warning_message;
  };

  DefaultPlanner(
    const DefaultPlannerParameters & param,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

  void set_map(const LaneletMapBin & msg);
  [[nodiscard]] bool ready() const;
  PlanResult plan(const RoutePoints & points);
  void updateRoute(const LaneletRoute & route);
  void clearRoute();
  [[nodiscard]] MarkerArray visualize(
    const LaneletRoute & route, float goal_lanelet_transparency = 0.05) const;
  [[nodiscard]] static MarkerArray visualize_debug_footprint(
    autoware_utils::LinearRing2d goal_footprint);
  [[nodiscard]] const autoware::route_handler::RouteHandler & getRouteHandler() const
  {
    return route_handler_;
  }
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

protected:
  using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
  using Pose = geometry_msgs::msg::Pose;
  bool is_graph_ready_;
  autoware::route_handler::RouteHandler route_handler_;

  DefaultPlannerParameters param_;

  // goal_footprint is only set once is_goal_valid() reaches the footprint computation (see
  // PlanResult::goal_footprint).
  struct GoalValidationResult
  {
    bool is_valid;
    std::optional<autoware_utils::LinearRing2d> goal_footprint;
    std::optional<std::string> warning_message;
  };

  /**
   * @brief check if the goal_footprint is within the lanelets closest to the goal plus the
   * succeeding lanelets around the goal
   * @attention this function will terminate when the accumulated search length from the initial
   * current_lanelet exceeds max_longitudinal_offset_m + search_margin, so under normal assumptions
   * (i.e. the map is composed of finite elements of practically normal sized lanelets), it is
   * assured to terminate
   * @param goal_lanelets the lanelets closest to and around the goal
   * @param goal_footprint footprint of the ego vehicle at the goal pose
   */
  [[nodiscard]] bool check_goal_footprint_inside_lanes(
    const lanelet::ConstLanelets & lanelets_near_goal,
    const autoware_utils::Polygon2d & goal_footprint) const;

  /**
   * @brief return true if (1)the goal is in parking area or (2)the goal is on the lanes and the
   * footprint around the goal does not overlap the lanes
   */
  GoalValidationResult is_goal_valid(const geometry_msgs::msg::Pose & goal);

  /**
   * @brief project the specified goal pose onto the goal lanelet(the last preferred lanelet of
   * route_sections) and return the z-aligned goal position
   */
  Pose refine_goal_height(const Pose & goal, const RouteSections & route_sections);
};

}  // namespace autoware::mission_planner_universe::lanelet2

#endif  // AUTOWARE__MISSION_PLANNER_UNIVERSE__DEFAULT_PLANNER_HPP_
