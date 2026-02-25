// Copyright 2026 TIER IV, Inc.
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

#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include "type_alias.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

struct RouteContext
{
  lanelet::LaneletMapPtr lanelet_map_ptr{nullptr};
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr{nullptr};
  lanelet::routing::RoutingGraphPtr routing_graph_ptr{nullptr};

  std::string route_frame_id{};
  geometry_msgs::msg::Pose goal_pose{};

  lanelet::ConstLanelets route_lanelets{};
  lanelet::ConstLanelets preferred_lanelets{};
  lanelet::ConstLanelets start_lanelets{};
  lanelet::ConstLanelets goal_lanelets{};

  std::optional<lanelet::ConstLanelet> closest_preferred_lanelet;
};

struct WaypointGroup
{
  struct Waypoint
  {
    lanelet::ConstPoint3d point;
    lanelet::Id lane_id;
  };

  struct Interval
  {
    double start;
    double end;
  };

  std::vector<Waypoint> waypoints;
  Interval interval;
};

template <typename T>
struct PathRange
{
  T left;
  T right;
};

struct TrajectoryShiftParams
{
  double minimum_shift_length{0.1};      // [m] lateral offset threshold to trigger shift
  double minimum_shift_yaw{0.1};         // [rad] yaw deviation threshold to trigger shift
  double minimum_shift_distance{5.0};    // [m] floor for shift distance
  double min_speed_for_curvature{2.77};  // [m/s] lower bound on speed for kappa0 computation
  double lateral_accel_limit{0.5};       // [m/s^2] allowed lateral acceleration budget
};

// ---------------------------------------------------------------------------
// PathPlanner class
// ---------------------------------------------------------------------------

class PathPlanner
{
public:
  PathPlanner(
    const rclcpp::Logger & logger, rclcpp::Clock::SharedPtr clock,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper, const Params & params,
    const VehicleInfo & vehicle_info);

  // Route / map initialisation
  void set_planner_data(
    const LaneletMapBin::ConstSharedPtr & lanelet_map_bin_ptr,
    const LaneletRoute::ConstSharedPtr & route_ptr);

  // State management
  void set_route_context(const RouteContext & route_context);
  RouteContext & route_context();
  const RouteContext & route_context() const;

  bool update_current_lanelet(const geometry_msgs::msg::Pose & current_pose);

  // Path planning
  std::optional<PathWithLaneId> plan_path(const geometry_msgs::msg::Pose & current_pose);
  std::optional<PathWithLaneId> generate_path(
    const lanelet::LaneletSequence & lanelet_sequence, double s_start, double s_end);

  // Trajectory shifting
  Trajectory shift_trajectory_to_ego(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose, double ego_velocity,
    double ego_yaw_rate, const TrajectoryShiftParams & shift_params, double delta_arc_length);

  // Path to trajectory conversion
  Trajectory convert_path_to_trajectory(const PathWithLaneId & path, double resample_interval);

  // Params update
  void update_params(const Params & params);

private:
  void set_route(const LaneletRoute::ConstSharedPtr & route_ptr);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  Params params_;
  VehicleInfo vehicle_info_;
  RouteContext route_context_;
  std::optional<lanelet::ConstLanelet> current_lanelet_;
};

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------
namespace utils
{

/**
 * @brief get lanelets within route that are in specified distance backward from target lanelet
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route_up_to(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data, const double distance);

/**
 * @brief get lanelets within route that are in specified distance forward from target lanelet
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route_after(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data, const double distance);

/**
 * @brief get previous lanelet within route
 */
std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data);

/**
 * @brief get next lanelet within route
 */
std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data);

/**
 * @brief get waypoints in lanelet sequence and group them
 */
std::vector<WaypointGroup> get_waypoint_groups(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::LaneletMap & lanelet_map,
  const double group_separation_threshold, const double interval_margin_ratio);

/**
 * @brief get position of first intersection (including self-intersection) in lanelet sequence
 */
std::optional<double> get_first_intersection_arc_length(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const double vehicle_length);

/**
 * @brief get position of first self-intersection of line string in arc length
 */
std::optional<double> get_first_self_intersection_arc_length(
  const lanelet::BasicLineString2d & line_string);

/**
 * @brief get position of given point on centerline projected to path in arc length
 */
double get_arc_length_on_path(
  const lanelet::LaneletSequence & lanelet_sequence, const std::vector<PathPointWithLaneId> & path,
  const double s_centerline);

/**
 * @brief get path bounds for PathWithLaneId cropped within specified range
 */
PathRange<std::vector<geometry_msgs::msg::Point>> get_path_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end);

/**
 * @brief crop line string
 */
std::vector<geometry_msgs::msg::Point> crop_line_string(
  const std::vector<geometry_msgs::msg::Point> & line_string, const double s_start,
  const double s_end);

/**
 * @brief get positions of given point on centerline projected to left / right bound in arc length
 */
PathRange<double> get_arc_length_on_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_centerline);

/**
 * @brief get positions of given point on left / right bound projected to centerline in arc length
 */
PathRange<std::optional<double>> get_arc_length_on_centerline(
  const lanelet::LaneletSequence & lanelet_sequence, const std::optional<double> & s_left_bound,
  const std::optional<double> & s_right_bound);

/**
 * @brief Recreate the path with a given goal pose
 */
PathPointTrajectory refine_path_for_goal(
  const PathPointTrajectory & input, const geometry_msgs::msg::Pose & goal_pose,
  const lanelet::Id goal_lane_id, const double search_radius_range, const double pre_goal_offset);

/**
 * @brief Extract lanelets from the trajectory
 */
lanelet::ConstLanelets extract_lanelets_from_trajectory(
  const PathPointTrajectory & trajectory, const RouteContext & planner_data);

/**
 * @brief Check if the pose is in the lanelets
 */
bool is_in_lanelets(const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanes);

/**
 * @brief Check if the trajectory is inside the lanelets
 */
bool is_trajectory_inside_lanelets(
  const PathPointTrajectory & refined_path, const lanelet::ConstLanelets & lanelets);

/**
 * @brief Modify path for smooth goal connection
 */
std::optional<PathPointTrajectory> modify_path_for_smooth_goal_connection(
  const PathPointTrajectory & trajectory, const RouteContext & planner_data,
  const double search_radius_range, const double pre_goal_offset);

}  // namespace utils
}  // namespace autoware::minimum_rule_based_planner

#endif  // PATH_PLANNER_HPP_
