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

std::vector<size_t> getCuspPointIndices(const PathWithLaneId & path, const double angle_threshold_deg)
{
  std::vector<size_t> cusp_indices;
  if (path.points.size() < 2) {
    return cusp_indices;
  }

  const double angle_threshold_rad = autoware_utils::deg2rad(angle_threshold_deg);

  // Detect cusp points by comparing yaw angles between consecutive path points
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & prev_point = path.points[i - 1].point.pose;
    const auto & curr_point = path.points[i].point.pose;

    // Extract yaw angles from path point orientations
    const double prev_yaw = tf2::getYaw(prev_point.orientation);
    const double curr_yaw = tf2::getYaw(curr_point.orientation);

    // Calculate normalized angle difference
    const double angle_diff = autoware_utils::normalize_radian(curr_yaw - prev_yaw);

    // If angle change exceeds threshold, mark as cusp point
    if (std::abs(angle_diff) > angle_threshold_rad) {
      cusp_indices.push_back(i);
    }
  }
  return cusp_indices;
}

std::vector<CuspPoint> detectCuspPointsFromPath(
  const PathWithLaneId & path, const double angle_threshold_deg)
{
  const auto cusp_indices = getCuspPointIndices(path, angle_threshold_deg);
  std::vector<CuspPoint> cusp_points;
  cusp_points.reserve(cusp_indices.size());
  for (const auto idx : cusp_indices) {
    CuspPoint cusp_point;
    cusp_point.pose = path.points.at(idx).point.pose;
    cusp_points.push_back(cusp_point);
  }
  return cusp_points;
}

bool isSameCuspPoint(
  const CuspPoint & a, const CuspPoint & b, const double position_threshold_m)
{
  return autoware_utils::calc_distance2d(a.pose.position, b.pose.position) < position_threshold_m;
}

void mergeNewCuspPointsAheadOfEgo(
  std::vector<CuspPoint> & tracked_cusp_points,
  const std::vector<CuspPoint> & detected_cusp_points, const PathWithLaneId & path,
  const geometry_msgs::msg::Pose & ego_pose, const double dedup_distance_m)
{
  if (detected_cusp_points.empty()) {
    return;
  }

  const auto ego_idx_opt = autoware::motion_utils::findNearestIndex(path.points, ego_pose);

  for (const auto & candidate : detected_cusp_points) {
    const auto candidate_idx_opt =
      autoware::motion_utils::findNearestIndex(path.points, candidate.pose);
    if (ego_idx_opt && candidate_idx_opt && *candidate_idx_opt <= *ego_idx_opt) {
      continue;
    }

    const bool already_tracked = std::any_of(
      tracked_cusp_points.begin(), tracked_cusp_points.end(), [&](const CuspPoint & tracked) {
        return isSameCuspPoint(tracked, candidate, dedup_distance_m);
      });
    if (!already_tracked) {
      tracked_cusp_points.push_back(candidate);
    }
  }
}

double calcDistanceAlongPathToPose(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  const geometry_msgs::msg::Pose & target_pose)
{
  if (path.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto ego_idx_opt =
    autoware::motion_utils::findNearestIndex(path.points, ego_pose);
  const size_t target_idx =
    autoware::motion_utils::findNearestIndex(path.points, target_pose.position);
  if (!ego_idx_opt) {
    return autoware_utils::calc_distance2d(ego_pose.position, target_pose.position);
  }

  if (target_idx >= *ego_idx_opt) {
    return autoware::motion_utils::calcSignedArcLength(
      path.points, *ego_idx_opt, target_idx);
  }

  return autoware_utils::calc_distance2d(ego_pose.position, target_pose.position);
}

PathWithLaneId getReferencePathFromDirectionChangeLanelets(
  const PathWithLaneId & path,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  [[maybe_unused]] const DirectionChangeParameters & parameters)
{
  PathWithLaneId out;
  if (!route_handler || path.points.empty()) {
    return out;
  }
  std::vector<int64_t> ordered_lane_ids;
  for (const auto & pt : path.points) {
    for (const auto & lane_id : pt.lane_ids) {
      try {
        route_handler->getLaneletsFromId(lane_id);
        if (ordered_lane_ids.empty() || ordered_lane_ids.back() != lane_id) {
          ordered_lane_ids.push_back(lane_id);
        }
      } catch (...) {
        continue;
      }
    }
  }
  if (ordered_lane_ids.empty()) {
    return out;
  }
  lanelet::ConstLanelets lanelet_sequence;
  lanelet_sequence.reserve(ordered_lane_ids.size());
  for (const auto & id : ordered_lane_ids) {
    try {
      lanelet_sequence.push_back(route_handler->getLaneletsFromId(id));
    } catch (...) {
      return out;
    }
  }
  const auto raw_path =
    route_handler->getCenterLinePath(lanelet_sequence, 0.0, std::numeric_limits<double>::max());
  if (raw_path.points.empty()) {
    return out;
  }
  out = raw_path;
  out.header = route_handler->getRouteHeader();

  std::cout << "[Debug2 DirectionChangeModule] getReferencePathFromDirectionChangeLanelets: out.points.size(): " << out.points.size() << std::endl;
  std::cout << "[Debug2 DirectionChangeModule] getReferencePathFromDirectionChangeLanelets: lanelets: " << lanelet_sequence << std::endl;

  try {
    const auto goal_pose = route_handler->getGoalPose();
    const auto goal_idx_opt =
      autoware::motion_utils::findNearestIndex(out.points, goal_pose.position);
    if (goal_idx_opt >= out.points.size()) {
      return out;
    }

    out.points.resize(goal_idx_opt + 1);
  } catch (...) {
    // No goal or getGoalPose failed; keep full path
  }

  return out;
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

bool isEgoDrivingForwardWrtLane(
  const geometry_msgs::msg::Pose & ego_pose, const PathWithLaneId & reference_path)
{
  if (reference_path.points.empty()) {
    return true;
  }

  const auto nearest_idx_opt =
    autoware::motion_utils::findNearestIndex(reference_path.points, ego_pose);
  if (!nearest_idx_opt) {
    return true;
  }

  const double ref_yaw =
    tf2::getYaw(reference_path.points.at(*nearest_idx_opt).point.pose.orientation);
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  return std::abs(autoware_utils::normalize_radian(ego_yaw - ref_yaw)) < M_PI_2;
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

namespace
{
void flipPathPointYawByPi(
  autoware_internal_planning_msgs::msg::PathPointWithLaneId & point)
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
