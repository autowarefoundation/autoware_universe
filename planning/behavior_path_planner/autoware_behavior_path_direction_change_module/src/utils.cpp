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

#include "autoware/behavior_path_direction_change_module/utils.hpp"

#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace
{
constexpr double kMinLateralShiftM = 0.05;
constexpr double kMinManeuverLengthM = 0.0;
constexpr double kShiftValidationSampleIntervalM = 1.0;

struct CubicShiftCoefficients
{
  double a2{0.0};
  double a3{0.0};
};

CubicShiftCoefficients computeCubicShiftCoefficients(
  const double d_goal, const double L, const double goal_yaw_delta)
{
  // d(s) = a2*s^2 + a3*s^3 with d(0)=0, d'(0)=0, d(L)=d_goal, d'(L)=tan(goal_yaw_delta)
  const double L2 = L * L;
  const double L3 = L2 * L;
  const double tan_goal_yaw_delta = std::tan(goal_yaw_delta);
  CubicShiftCoefficients coeffs;
  coeffs.a2 = 3.0 * d_goal / L2 - tan_goal_yaw_delta / L;
  coeffs.a3 = -2.0 * d_goal / L3 + tan_goal_yaw_delta / L2;
  return coeffs;
}

double evalShift(const CubicShiftCoefficients & coeffs, const double s)
{
  return coeffs.a2 * s * s + coeffs.a3 * s * s * s;
}

double evalShiftDerivative(const CubicShiftCoefficients & coeffs, const double s)
{
  return 2.0 * coeffs.a2 * s + 3.0 * coeffs.a3 * s * s;
}

double computeManeuverLength(const double lateral_shift, const double max_allowed_yaw_rad)
{
  // Approximation: tan(theta_max) ~= lateral_shift / maneuver_length
  // Assumption: max_allowed_yaw_rad > 0.0

  return std::max(kMinManeuverLengthM, std::abs(lateral_shift) / std::tan(max_allowed_yaw_rad));
}

geometry_msgs::msg::Pose applyLateralShiftToPose(
  const geometry_msgs::msg::Pose & ref_pose, const double d, const double d_derivative)
{
  const double yaw_ref = tf2::getYaw(ref_pose.orientation);
  geometry_msgs::msg::Pose shifted = ref_pose;
  shifted.position.x -= std::sin(yaw_ref) * d;
  shifted.position.y += std::cos(yaw_ref) * d;
  const double yaw = autoware_utils::normalize_radian(yaw_ref + std::atan(d_derivative));
  shifted.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  return shifted;
}

lanelet::ConstLanelets collectLaneletsForPoint(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  lanelet::ConstLanelets lanelets;
  if (!route_handler) {
    return lanelets;
  }
  for (const auto lane_id : point.lane_ids) {
    try {
      lanelets.push_back(route_handler->getLaneletsFromId(lane_id));
    } catch (...) {
      continue;
    }
  }
  return lanelets;
}

bool isPoseInsideLanelets(
  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanelets)
{
  if (lanelets.empty()) {
    return false;
  }
  return utils::isInLanelets(pose, lanelets);
}

bool validateShiftedPathInCorridor(
  const PathWithLaneId & reference_path, const std::vector<double> & reference_arc_lengths,
  const CubicShiftCoefficients & coeffs, const double shift_start_s, const double goal_s,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  if (
    reference_path.points.size() < 2 ||
    reference_arc_lengths.size() != reference_path.points.size()) {
    return false;
  }

  for (double sample_s = shift_start_s; sample_s <= goal_s + 1e-3;
       sample_s += kShiftValidationSampleIntervalM) {
    // Find segment containing sample_s for lane association and reference interpolation.
    size_t seg_idx = 0;
    for (size_t i = 1; i < reference_arc_lengths.size(); ++i) {
      if (reference_arc_lengths.at(i) >= sample_s) {
        seg_idx = i - 1;
        break;
      }
      seg_idx = i - 1;
    }

    const double s0 = reference_arc_lengths.at(seg_idx);
    const double s1 = reference_arc_lengths.at(seg_idx + 1);
    const double ratio = (s1 - s0) > 1e-6 ? (sample_s - s0) / (s1 - s0) : 0.0;

    const auto & p0 = reference_path.points.at(seg_idx).point.pose;
    const auto & p1 = reference_path.points.at(seg_idx + 1).point.pose;
    geometry_msgs::msg::Pose ref_pose;
    ref_pose.position.x = p0.position.x + ratio * (p1.position.x - p0.position.x);
    ref_pose.position.y = p0.position.y + ratio * (p1.position.y - p0.position.y);
    ref_pose.position.z = p0.position.z + ratio * (p1.position.z - p0.position.z);
    const double yaw0 = tf2::getYaw(p0.orientation);
    const double yaw1 = tf2::getYaw(p1.orientation);
    const double yaw_ref = autoware_utils::normalize_radian(yaw0 + ratio * (yaw1 - yaw0));
    ref_pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw_ref);

    const double local_s = sample_s - shift_start_s;
    const double d = evalShift(coeffs, local_s);
    const double d_derivative = evalShiftDerivative(coeffs, local_s);
    const auto shifted_pose = applyLateralShiftToPose(ref_pose, d, d_derivative);

    const auto lanelets = collectLaneletsForPoint(reference_path.points.at(seg_idx), route_handler);
    if (!isPoseInsideLanelets(shifted_pose, lanelets)) {
      return false;
    }
  }

  return true;
}
/*
void snapPathEndToGoal(PathWithLaneId * path, const geometry_msgs::msg::Pose & goal_pose)
{
  if (!path || path->points.empty()) {
    return;
  }
  auto & goal_point = path->points.back().point;
  goal_point.pose.position = goal_pose.position;
  // goal_point.pose.orientation = goal_pose.orientation;
  goal_point.longitudinal_velocity_mps = 0.0;
} */
}  // namespace

std::vector<CuspPoint> detectCuspPointsOnPathWithIndices(
  const PathWithLaneId & path, const double angle_threshold_deg)
{
  std::vector<CuspPoint> cusp_points;
  if (path.points.size() < 2) {
    return cusp_points;
  }

  const double angle_threshold_rad = autoware_utils::deg2rad(angle_threshold_deg);
  for (size_t i = 1; i < path.points.size(); ++i) {
    const double prev_yaw = tf2::getYaw(path.points.at(i - 1).point.pose.orientation);
    const double curr_yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    if (std::abs(autoware_utils::normalize_radian(curr_yaw - prev_yaw)) > angle_threshold_rad) {
      CuspPoint cusp_point;
      cusp_point.tagged_centerline_index = i;
      cusp_point.pose = path.points.at(i).point.pose;
      cusp_points.push_back(cusp_point);
    }
  }
  return cusp_points;
}

double calcDistanceAlongPathToPose(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  const geometry_msgs::msg::Pose & target_pose)
{
  if (path.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto ego_idx_opt = autoware::motion_utils::findNearestIndex(path.points, ego_pose);
  const size_t target_idx =
    autoware::motion_utils::findNearestIndex(path.points, target_pose.position);
  if (!ego_idx_opt) {
    return autoware_utils::calc_distance2d(ego_pose.position, target_pose.position);
  }

  if (target_idx >= *ego_idx_opt) {
    return autoware::motion_utils::calcSignedArcLength(path.points, *ego_idx_opt, target_idx);
  }

  return autoware_utils::calc_distance2d(ego_pose.position, target_pose.position);
}

namespace
{
bool pathPointHasAnyLaneId(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point,
  const std::set<int64_t> & lane_id_set)
{
  return std::any_of(point.lane_ids.begin(), point.lane_ids.end(), [&](const int64_t lane_id) {
    return lane_id_set.count(lane_id) > 0;
  });
}

lanelet::ConstLanelets laneletsFromIds(
  const std::vector<int64_t> & lane_ids,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(lane_ids.size());
  for (const auto lane_id : lane_ids) {
    try {
      lanelets.push_back(route_handler->getLaneletsFromId(lane_id));
    } catch (...) {
      return {};
    }
  }
  return lanelets;
}

PathWithLaneId trimPathToGoal(
  PathWithLaneId path, const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  try {
    const auto goal_pose = route_handler->getGoalPose();
    const auto goal_idx = autoware::motion_utils::findNearestIndex(path.points, goal_pose.position);
    if (goal_idx < path.points.size()) {
      path.points.resize(goal_idx + 1);
    }
  } catch (...) {
    // No goal or getGoalPose failed; keep full path
  }
  return path;
}

PathWithLaneId extractPathPointsForLaneIds(
  const PathWithLaneId & path, const std::vector<int64_t> & target_lane_ids)
{
  PathWithLaneId extracted_path;
  if (path.points.empty() || target_lane_ids.empty()) {
    return extracted_path;
  }

  const std::set<int64_t> target_lane_id_set(target_lane_ids.begin(), target_lane_ids.end());
  extracted_path.header = path.header;
  extracted_path.points.reserve(path.points.size());
  for (const auto & point : path.points) {
    if (pathPointHasAnyLaneId(point, target_lane_id_set)) {
      extracted_path.points.push_back(point);
    }
  }
  return extracted_path;
}

PathWithLaneId buildCenterlinePathForLaneIds(
  const std::vector<int64_t> & lane_ids,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  PathWithLaneId centerline_path;
  if (!route_handler || lane_ids.empty()) {
    return centerline_path;
  }

  const auto lanelets = laneletsFromIds(lane_ids, route_handler);
  if (lanelets.empty()) {
    return centerline_path;
  }

  centerline_path =
    route_handler->getCenterLinePath(lanelets, 0.0, std::numeric_limits<double>::max());
  if (centerline_path.points.empty()) {
    return centerline_path;
  }

  centerline_path.header = route_handler->getRouteHeader();
  return centerline_path;
}
}  // namespace

std::optional<DirectionChangeRouteContext> buildDirectionChangeRouteContext(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  DirectionChangeRouteContext context;
  if (!route_handler) {
    return std::nullopt;
  }

  const auto route_lanelets = route_handler->getPreferredLanelets();
  if (route_lanelets.empty()) {
    return std::nullopt;
  }

  context.route_lanelet_ids_ordered.reserve(route_lanelets.size());
  for (const auto & lanelet : route_lanelets) {
    context.route_lanelet_ids_ordered.push_back(lanelet.id());
    if (hasDirectionChangeAreaTag(lanelet)) {
      context.tagged_lanelet_ids_ordered.push_back(lanelet.id());
    }
  }

  if (context.tagged_lanelet_ids_ordered.empty()) {
    return std::nullopt;
  }

  const auto first_tagged_id = context.tagged_lanelet_ids_ordered.front();
  const auto last_tagged_id = context.tagged_lanelet_ids_ordered.back();

  bool reached_first_tagged = false;
  bool passed_last_tagged = false;
  for (const auto route_lanelet_id : context.route_lanelet_ids_ordered) {
    if (route_lanelet_id == first_tagged_id) {
      reached_first_tagged = true;
    }
    if (!reached_first_tagged) {
      context.prefix_lanelet_ids.push_back(route_lanelet_id);
    }
    if (passed_last_tagged) {
      context.suffix_lanelet_ids.push_back(route_lanelet_id);
    }
    if (route_lanelet_id == last_tagged_id) {
      passed_last_tagged = true;
    }
  }

  const auto tagged_lanelets = laneletsFromIds(context.tagged_lanelet_ids_ordered, route_handler);
  if (tagged_lanelets.empty()) {
    return std::nullopt;
  }

  auto tagged_centerline_path =
    route_handler->getCenterLinePath(tagged_lanelets, 0.0, std::numeric_limits<double>::max());
  if (tagged_centerline_path.points.empty()) {
    return std::nullopt;
  }

  tagged_centerline_path.header = route_handler->getRouteHeader();
  context.tagged_lanelet_centerline_path =
    trimPathToGoal(std::move(tagged_centerline_path), route_handler);
  context.is_valid = !context.tagged_lanelet_centerline_path.points.empty();
  return context.is_valid ? std::optional<DirectionChangeRouteContext>{context} : std::nullopt;
}

PathWithLaneId buildPathForLaneIds(
  const PathWithLaneId & previous_module_path, const std::vector<int64_t> & lane_ids,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  if (lane_ids.empty()) {
    return PathWithLaneId{};
  }

  auto path = extractPathPointsForLaneIds(previous_module_path, lane_ids);
  if (path.points.empty()) {
    path = buildCenterlinePathForLaneIds(lane_ids, route_handler);
  }
  return path;
}

ReferencePathAssemblyPhase determineReferencePathAssemblyPhase(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::msg::Pose & ego_pose, const DirectionChangeRouteContext & route_context,
  const bool all_cusps_visited)
{
  if (!route_handler || !route_context.is_valid) {
    return ReferencePathAssemblyPhase::INSIDE_TAGGED_CORRIDOR;
  }

  if (all_cusps_visited && !route_context.suffix_lanelet_ids.empty()) {
    return ReferencePathAssemblyPhase::EXITING_TAGGED_AREA;
  }

  lanelet::ConstLanelet closest_route_lanelet;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &closest_route_lanelet)) {
    return ReferencePathAssemblyPhase::INSIDE_TAGGED_CORRIDOR;
  }

  if (isEgoOnRouteLanelets(ego_pose, route_handler, route_context.tagged_lanelet_ids_ordered)) {
    return ReferencePathAssemblyPhase::INSIDE_TAGGED_CORRIDOR;
  }
  if (isEgoOnRouteLanelets(ego_pose, route_handler, route_context.prefix_lanelet_ids)) {
    return ReferencePathAssemblyPhase::APPROACHING_TAGGED_AREA;
  }
  if (isEgoOnRouteLanelets(ego_pose, route_handler, route_context.suffix_lanelet_ids)) {
    return ReferencePathAssemblyPhase::EXITING_TAGGED_AREA;
  }

  if (hasDirectionChangeAreaTag(closest_route_lanelet)) {
    return ReferencePathAssemblyPhase::INSIDE_TAGGED_CORRIDOR;
  }

  return ReferencePathAssemblyPhase::INSIDE_TAGGED_CORRIDOR;
}

PathWithLaneId assembleReferencePathWithLaneStitching(
  const DirectionChangeRouteContext & route_context, const PathWithLaneId & previous_module_path,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const ReferencePathAssemblyPhase assembly_phase)
{
  const auto & tagged_centerline = route_context.tagged_lanelet_centerline_path;
  if (!route_context.is_valid || tagged_centerline.points.empty()) {
    return previous_module_path;
  }

  const auto prefix_path =
    buildPathForLaneIds(previous_module_path, route_context.prefix_lanelet_ids, route_handler);
  const auto suffix_path =
    buildPathForLaneIds(previous_module_path, route_context.suffix_lanelet_ids, route_handler);

  switch (assembly_phase) {
    case ReferencePathAssemblyPhase::APPROACHING_TAGGED_AREA:
      return prefix_path.points.empty()
               ? tagged_centerline
               : utils::combinePath(prefix_path, tagged_centerline);
    case ReferencePathAssemblyPhase::INSIDE_TAGGED_CORRIDOR:
      return tagged_centerline;
    case ReferencePathAssemblyPhase::EXITING_TAGGED_AREA:
      return suffix_path.points.empty()
               ? tagged_centerline
               : utils::combinePath(tagged_centerline, suffix_path);
  }

  return tagged_centerline;
}

PathWithLaneId slicePathBetweenCuspIndices(
  const PathWithLaneId & path, const std::optional<size_t> start_after_cusp_index,
  const size_t end_cusp_index)
{
  PathWithLaneId sliced_path;
  if (path.points.empty() || end_cusp_index >= path.points.size()) {
    return sliced_path;
  }

  size_t start_idx = 0;
  if (start_after_cusp_index) {
    start_idx = std::min(*start_after_cusp_index + 1, path.points.size() - 1);
  }

  size_t end_idx = end_cusp_index;
  if (start_idx >= end_idx) {
    start_idx = end_idx > 0 ? end_idx - 1 : 0;
  }
  if (start_idx >= end_idx && path.points.size() > 1) {
    end_idx = std::min(start_idx + 1, path.points.size());
  }
  if (start_idx >= end_idx) {
    return sliced_path;
  }

  sliced_path.header = path.header;
  sliced_path.points.assign(
    path.points.begin() + static_cast<std::ptrdiff_t>(start_idx),
    path.points.begin() + static_cast<std::ptrdiff_t>(end_idx));
  return sliced_path;
}

PathWithLaneId slicePathToGoalFromCuspIndex(
  const PathWithLaneId & path, const std::optional<size_t> start_after_cusp_index,
  const geometry_msgs::msg::Pose & goal_pose)
{
  PathWithLaneId sliced_path;
  if (path.points.empty()) {
    return sliced_path;
  }

  const size_t goal_idx = autoware::motion_utils::findNearestIndex(path.points, goal_pose.position);
  if (goal_idx >= path.points.size()) {
    return sliced_path;
  }

  size_t start_idx = 0;
  if (start_after_cusp_index) {
    start_idx = std::min(*start_after_cusp_index + 1, path.points.size() - 1);
  }

  const size_t end_idx = goal_idx + 1;
  if (start_idx >= end_idx) {
    start_idx = 0;
  }

  sliced_path.header = path.header;
  sliced_path.points.assign(
    path.points.begin() + static_cast<std::ptrdiff_t>(start_idx),
    path.points.begin() + static_cast<std::ptrdiff_t>(end_idx));
  return sliced_path;
}

bool isEgoNearRouteGoal(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const double th_arrived_distance, const std::vector<int64_t> & suffix_lanelet_ids)
{
  if (!route_handler) {
    return false;
  }

  geometry_msgs::msg::Pose goal_pose;
  try {
    goal_pose = route_handler->getGoalPose();
  } catch (...) {
    return false;
  }

  const double dist_to_goal =
    autoware_utils::calc_distance2d(ego_pose.position, goal_pose.position);
  if (dist_to_goal < th_arrived_distance) {
    return true;
  }

  const double goal_yaw = tf2::getYaw(goal_pose.orientation);
  const double dx = ego_pose.position.x - goal_pose.position.x;
  const double dy = ego_pose.position.y - goal_pose.position.y;
  const double longitudinal = dx * std::cos(goal_yaw) + dy * std::sin(goal_yaw);
  const double lateral = -dx * std::sin(goal_yaw) + dy * std::cos(goal_yaw);
  constexpr double k_longitudinal_tolerance_multiplier = 3.0;
  if (
    std::abs(longitudinal) < th_arrived_distance * k_longitudinal_tolerance_multiplier &&
    std::abs(lateral) < th_arrived_distance) {
    return true;
  }

  lanelet::ConstLanelet closest_route_lanelet;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &closest_route_lanelet)) {
    return false;
  }

  const int64_t ego_lane_id = closest_route_lanelet.id();
  const auto is_on_suffix_lane = [&]() {
    return std::find(suffix_lanelet_ids.begin(), suffix_lanelet_ids.end(), ego_lane_id) !=
           suffix_lanelet_ids.end();
  };

  if (
    is_on_suffix_lane() &&
    dist_to_goal < th_arrived_distance * k_longitudinal_tolerance_multiplier) {
    return true;
  }

  if (
    route_handler->isInGoalRouteSection(closest_route_lanelet) &&
    dist_to_goal < th_arrived_distance * k_longitudinal_tolerance_multiplier) {
    return true;
  }

  return false;
}

bool isEgoOnRouteLanelets(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const std::vector<int64_t> & lanelet_ids)
{
  if (!route_handler || lanelet_ids.empty()) {
    return false;
  }

  lanelet::ConstLanelet closest_route_lanelet;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &closest_route_lanelet)) {
    return false;
  }

  const int64_t ego_lane_id = closest_route_lanelet.id();
  return std::find(lanelet_ids.begin(), lanelet_ids.end(), ego_lane_id) != lanelet_ids.end();
}

bool isDirectionChangeManeuverFinished(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const DirectionChangeRouteContext & route_context, const double th_arrived_distance,
  const bool all_cusps_visited)
{
  if (!all_cusps_visited || !route_context.is_valid) {
    return false;
  }

  const bool near_goal = isEgoNearRouteGoal(
    ego_pose, route_handler, th_arrived_distance, route_context.suffix_lanelet_ids);
  const bool on_tagged = isEgoOnRouteLanelets(
    ego_pose, route_handler, route_context.tagged_lanelet_ids_ordered);

  return near_goal || !on_tagged;
}

std::optional<PathWithLaneId> applyGoalLateralShift(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & goal_pose,
  const DirectionChangeParameters & parameters,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  static const auto logger_ = rclcpp::get_logger("direction_change");

  const auto goal_idx_opt =
    autoware::motion_utils::findNearestIndex(path.points, goal_pose.position);
  if (goal_idx_opt >= path.points.size()) {
    return std::nullopt;
  }
  const size_t goal_idx = goal_idx_opt;

  const auto arc_lengths = utils::calcPathArcLengthArray(path);
  const double goal_s = arc_lengths.at(goal_idx);

  const double d_goal =
    autoware::motion_utils::calcLateralOffset(path.points, goal_pose.position, false);

  const double ref_yaw_at_goal = tf2::getYaw(path.points.at(goal_idx).point.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_pose.orientation);
  const double goal_yaw_delta = autoware_utils::normalize_radian(goal_yaw - ref_yaw_at_goal);

  const double max_allowed_yaw_rad = autoware_utils::deg2rad(parameters.max_allowed_yaw_deg);
  const double maneuver_length = computeManeuverLength(d_goal, max_allowed_yaw_rad);

  const double shift_start_s = goal_s - maneuver_length;

  if (shift_start_s < 0.0) {
    RCLCPP_WARN(
      logger_,
      "Infeasible lateral shift request. "
      "Required maneuver length: %.2f m, "
      "available distance: %.2f m",
      maneuver_length, goal_s);

    return std::nullopt;
  }
  // Compute cubic shift coefficients
  const auto coeffs = computeCubicShiftCoefficients(d_goal, maneuver_length, goal_yaw_delta);

  PathWithLaneId shifted_path = path;
  for (size_t i = 0; i < shifted_path.points.size(); ++i) {
    const double s = arc_lengths.at(i);
    if (s < shift_start_s - 1e-6 || s > goal_s + 1e-6) {
      continue;
    }

    const double local_s = s - shift_start_s;
    const double d = evalShift(coeffs, local_s);
    const double d_derivative = evalShiftDerivative(coeffs, local_s);
    shifted_path.points.at(i).point.pose =
      applyLateralShiftToPose(path.points.at(i).point.pose, d, d_derivative);
  }

  // snapPathEndToGoal(&shifted_path, goal_pose);

  if (!validateShiftedPathInCorridor(
        path, arc_lengths, coeffs, shift_start_s, goal_s, route_handler)) {
    return std::nullopt;
  }

  return shifted_path;
}

double calcDistanceToPathEnd(const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose)
{
  if (path.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto nearest_idx_opt = autoware::motion_utils::findNearestIndex(path.points, ego_pose);
  if (!nearest_idx_opt) {
    return std::numeric_limits<double>::max();
  }

  return autoware::motion_utils::calcSignedArcLength(
    path.points, *nearest_idx_opt, path.points.size() - 1);
}

void clipPathAroundEgo(
  PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  const double backward_path_length, const double forward_path_length)
{
  if (path.points.empty()) {
    return;
  }

  const auto ego_idx_opt = autoware::motion_utils::findNearestIndex(path.points, ego_pose);
  if (!ego_idx_opt) {
    return;
  }

  utils::clipPathLength(path, *ego_idx_opt, forward_path_length, backward_path_length);
}

namespace
{
void flipPathPointYawByPi(autoware_internal_planning_msgs::msg::PathPointWithLaneId & point)
{
  double yaw = tf2::getYaw(point.point.pose.orientation);
  yaw = autoware_utils::normalize_radian(yaw + M_PI);
  point.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
}
}  // namespace

void flipPathPointOrientation(PathWithLaneId & path)
{
  for (auto & p : path.points) {
    flipPathPointYawByPi(p);
    p.point.longitudinal_velocity_mps = -std::abs(p.point.longitudinal_velocity_mps);
  }
}

void setPathPointVelocityToZero(PathWithLaneId & path, const size_t point_count)
{
  if (path.points.empty() || point_count == 0) {
    return;
  }
  const size_t n = std::min(point_count, path.points.size());
  for (size_t k = 0; k < n; ++k) {
    path.points[path.points.size() - 1 - k].point.longitudinal_velocity_mps = 0.0;
  }
}

bool hasDirectionChangeAreaTag(const lanelet::ConstLanelet & lanelet)
{
  const std::string direction_change_tag = lanelet.attributeOr("direction_change", "none");
  return direction_change_tag == "yes";
}

}  // namespace autoware::behavior_path_planner
