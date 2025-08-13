// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__CLOTHOID_PULL_OUT_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__CLOTHOID_PULL_OUT_HPP_

#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_planner_base.hpp"
#include "autoware/behavior_path_start_planner_module/util.hpp"
#include "autoware_utils/system/time_keeper.hpp"

#include <autoware/boundary_departure_checker/boundary_departure_checker.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::boundary_departure_checker::BoundaryDepartureChecker;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

// Forward declarations for clothoid-related structures
struct ArcSegment;
struct ClothoidSegment;
struct CompositeArcPath;

/**
 * @brief Correct clothoid by rigid transformation (rotation, translation, scaling)
 * @param clothoid_points Clothoid points after transformation
 * @param original_segment Original arc segment
 * @param start_pose Starting pose of the segment
 * @return Corrected point sequence
 */
std::vector<geometry_msgs::msg::Point> correct_clothoid_by_rigid_transform(
  const std::vector<geometry_msgs::msg::Point> & clothoid_points,
  const ArcSegment & original_segment, const geometry_msgs::msg::Pose & start_pose);

/**
 * @brief Generate entry clothoid segment with yaw angles
 * @param segment Clothoid segment
 * @param start_pose Starting pose
 * @param num_points Number of points to generate
 * @return Pair of pose sequence and end pose
 */
std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generate_clothoid_entry_with_yaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points);

/**
 * @brief Generate circular segment with yaw angles
 * @param segment Circular segment
 * @param start_pose Starting pose
 * @param num_points Number of points to generate
 * @return Pair of pose sequence and end pose
 */
std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generate_circular_segment_with_yaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points);

/**
 * @brief Generate exit clothoid segment with yaw angles
 * @param segment Clothoid segment
 * @param start_pose Starting pose
 * @param num_points Number of points to generate
 * @return Pair of pose sequence and end pose
 */
std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generate_clothoid_exit_with_yaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points);

/**
 * @brief Generate clothoid path from segments
 * @param segments Vector of clothoid segments
 * @param point_interval Interval between points
 * @param start_pose Starting pose
 * @return Generated clothoid path points
 */
std::vector<geometry_msgs::msg::Point> generate_clothoid_path(
  const std::vector<ClothoidSegment> & segments, double point_interval,
  const geometry_msgs::msg::Pose & start_pose);

/**
 * @brief Convert ArcSegment to clothoid curve
 * @param arc_segment Arc segment to convert
 * @param start_pose Starting pose
 * @param A_min Minimum clothoid parameter A
 * @param L_min Minimum clothoid parameter L
 * @param point_interval Interval between points
 * @return Optional clothoid path points (nullopt if conversion fails)
 */
std::optional<std::vector<geometry_msgs::msg::Point>> convert_arc_to_clothoid(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose, double A_min,
  double L_min, double point_interval);

/**
 * @brief Improved clothoid conversion function with endpoint correction
 * @param arc_segment Arc segment to convert
 * @param start_pose Starting pose
 * @param initial_velocity Initial velocity
 * @param wheel_base Vehicle wheel base
 * @param max_steer_angle_rate Maximum steering angle rate
 * @param point_interval Interval between points
 * @return Optional clothoid path points with correction (nullopt if conversion fails)
 */
std::optional<std::vector<geometry_msgs::msg::Point>> convert_arc_to_clothoid_with_correction(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose,
  double initial_velocity, double wheel_base, double max_steer_angle_rate, double point_interval);

/**
 * @brief Create straight path to end pose
 * @param start_pose Starting pose
 * @param forward_distance Forward distance
 * @param backward_distance Backward distance
 * @param point_interval Interval between points
 * @return Straight path poses
 */
std::vector<geometry_msgs::msg::Pose> create_straight_path_to_end_pose(
  const geometry_msgs::msg::Pose & start_pose, double forward_distance, double backward_distance,
  double point_interval);

/**
 * @brief Calculate necessary longitudinal distance for circular path planning with clothoid
 * consideration
 * @param minimum_radius Minimum turning radius
 * @param initial_velocity Initial velocity for clothoid calculation
 * @param wheel_base Vehicle wheel base
 * @param max_steer_angle_rate Maximum steering angle rate
 * @param centerline_path Centerline path for target pose calculation
 * @param start_pose Starting pose
 * @return Calculated longitudinal distance
 */
double calc_necessary_longitudinal_distance(
  const double minimum_radius, const double initial_velocity, const double wheel_base,
  const double max_steer_angle_rate, const PathWithLaneId & centerline_path,
  const geometry_msgs::msg::Pose & start_pose);

/**
 * @brief Calculate circular path
 * @param start_pose Starting pose
 * @param longitudinal_distance Longitudinal distance
 * @param lateral_distance Lateral distance
 * @param angle_diff Angle difference
 * @param minimum_radius Minimum turning radius
 * @return Composite arc path
 */
std::optional<CompositeArcPath> calc_circular_path(
  const geometry_msgs::msg::Pose & start_pose, const double longitudinal_distance,
  const double lateral_distance, const double angle_diff, const double minimum_radius);

/**
 * @brief Create PathWithLaneId from clothoid paths
 * @param clothoid_paths Array of clothoid paths
 * @param velocity Initial velocity
 * @param target_velocity Target velocity
 * @param acceleration Acceleration
 * @param search_lanes Search lanes for lane_ids and z coordinate
 * @param route_handler Route handler
 * @return PathWithLaneId
 */
std::optional<PathWithLaneId> create_path_with_lane_id_from_clothoid_paths(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & clothoid_paths, double velocity,
  double target_velocity, double acceleration, const lanelet::ConstLanelets & search_lanes,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

/**
 * @brief Combine centerline path with clothoid path
 * @param clothoid_path Clothoid path
 * @param centerline_path Centerline path
 * @param target_pose Target pose
 * @return Combined PathWithLaneId
 */
PathWithLaneId combine_path_with_centerline(
  const PathWithLaneId & clothoid_path, const PathWithLaneId & centerline_path,
  const geometry_msgs::msg::Pose & target_pose);

class ClothoidPullOut : public PullOutPlannerBase
{
public:
  explicit ClothoidPullOut(
    rclcpp::Node & node, const StartPlannerParameters & parameters,
    std::shared_ptr<autoware_utils::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils::TimeKeeper>());

  PlannerType getPlannerType() const override { return PlannerType::CLOTHOID; };
  std::optional<PullOutPath> plan(
    const Pose & start_pose, const Pose & goal_pose,
    const std::shared_ptr<const PlannerData> & planner_data,
    PlannerDebugData & planner_debug_data) override;

  std::shared_ptr<BoundaryDepartureChecker> boundary_departure_checker_;

  friend class TestClothoidPullOut;

  // private:
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__CLOTHOID_PULL_OUT_HPP_
