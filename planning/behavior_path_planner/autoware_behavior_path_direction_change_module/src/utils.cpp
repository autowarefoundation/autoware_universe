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
// Prefer longer smoother shifts: multiply the analytically derived maneuver length.
// constexpr double kManeuverLengthSafetyFactor = 1.3;

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

double computeManeuverLength(
  const double lateral_shift, const double max_allowed_yaw_rad)
{
  // Approximation: tan(theta_max) ~= lateral_shift / maneuver_length
  // Assumption: max_allowed_yaw_rad > 0.0

  return std::max(
    kMinManeuverLengthM,
    std::abs(lateral_shift) / std::tan(max_allowed_yaw_rad));
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
  if (reference_path.points.size() < 2 || reference_arc_lengths.size() != reference_path.points.size()) {
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

void snapPathEndToGoal(PathWithLaneId * path, const geometry_msgs::msg::Pose & goal_pose)
{
  if (!path || path->points.empty()) {
    return;
  }
  auto & goal_point = path->points.back().point;
  goal_point.pose.position = goal_pose.position;
  // goal_point.pose.orientation = goal_pose.orientation;
  goal_point.longitudinal_velocity_mps = 0.0;
}
}  // namespace

std::vector<size_t> detectCuspPoints(const PathWithLaneId & path, const double angle_threshold_deg)
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
    const double yaw_prev = tf2::getYaw(prev_point.orientation);
    const double yaw_curr = tf2::getYaw(curr_point.orientation);

    // Calculate normalized angle difference
    const double angle_diff = autoware_utils::normalize_radian(yaw_curr - yaw_prev);

    // If angle change exceeds threshold, mark as cusp point
    if (std::abs(angle_diff) > angle_threshold_rad) {
      cusp_indices.push_back(i);
    }
  }
  return cusp_indices;
}

void reverseOrientationAtCusps(PathWithLaneId * path, const std::vector<size_t> & cusp_indices)
{
  if (path->points.empty() || cusp_indices.empty()) {
    return;
  }

  // Optimized: Single pass through path points with cusp index tracking
  // Complexity: O(n + m) where n = path points, m = cusp indices
  // Pattern: Before first cusp: original, After first cusp: reversed, After second: original, etc.
  // Toggle orientation at each cusp: original → reversed → original → reversed → ...
  size_t cusp_idx = 0;              // Current cusp index pointer
  bool is_reversed = false;         // Current reversal state
  size_t reversed_point_count = 0;  // Count points with reversed orientation

  for (size_t i = 0; i < path->points.size(); ++i) {
    // Check if we've passed any new cusp points
    while (cusp_idx < cusp_indices.size() && i > cusp_indices[cusp_idx]) {
      is_reversed = !is_reversed;  // Toggle at each cusp passed
      ++cusp_idx;
    }

    if (is_reversed) {
      // Reverse orientation: add π radians to yaw
      double yaw = tf2::getYaw(path->points[i].point.pose.orientation);
      yaw = autoware_utils::normalize_radian(yaw + M_PI);
      path->points[i].point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
      ++reversed_point_count;
    }
  }
}

std::vector<size_t> detectLaneBoundaries(const PathWithLaneId & path)
{
  std::vector<size_t> lane_boundary_indices;

  // TODO(shin.sato): Implementation postponed
  // This method should detect lane transitions by:
  // 1. Tracking lane_ids in path points
  // 2. Finding indices where lane_id changes
  // 3. Returning vector of transition point indices
  //
  // For now, return empty vector (placeholder)

  if (path.points.size() < 2) {
    return lane_boundary_indices;
  }

  // Placeholder: Check for lane_id changes
  // Actual implementation will be added in future
  // Optimized: Use set for O(1) lookup instead of nested loops
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & prev_lane_ids = path.points[i - 1].lane_ids;
    const auto & curr_lane_ids = path.points[i].lane_ids;

    // Quick check: if sizes differ, definitely a boundary
    if (prev_lane_ids.size() != curr_lane_ids.size()) {
      lane_boundary_indices.push_back(i);
      continue;
    }

    // Use set for efficient lookup (O(n) instead of O(n²))
    std::set<int64_t> prev_set(prev_lane_ids.begin(), prev_lane_ids.end());
    std::set<int64_t> curr_set(curr_lane_ids.begin(), curr_lane_ids.end());

    // If sets differ, it's a lane boundary
    if (prev_set != curr_set) {
      lane_boundary_indices.push_back(i);
    }
  }

  return lane_boundary_indices;
}

PathWithLaneId getReferencePathFromDirectionChangeLanelets(
  const PathWithLaneId & path,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const DirectionChangeParameters & parameters)
{
  PathWithLaneId out;
  if (!route_handler || path.points.empty()) {
    return out;
  }
  std::vector<int64_t> ordered_lane_ids;
  for (const auto & pt : path.points) {
    for (const auto & lane_id : pt.lane_ids) {
      try {
        const auto lanelet = route_handler->getLaneletsFromId(lane_id);
        /*if (!hasDirectionChangeAreaTag(lanelet)) {
          continue;
        }*/
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

  try {
    const auto goal_pose = route_handler->getGoalPose();
  const auto goal_idx_opt = autoware::motion_utils::findNearestIndex(out.points, goal_pose.position);
    if (goal_idx_opt >= out.points.size()) {
      return out;
    }

    out.points.resize(goal_idx_opt + 1);

    if (parameters.enable_goal_lateral_shift) {
      if (const auto shifted_path = applyGoalLateralShift(out, goal_pose, parameters, route_handler)) {
        return *shifted_path;
      }
    }

    // Fallback: hard snap when shift is disabled or validation fails.
    snapPathEndToGoal(&out, goal_pose);
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
  /*if (path.points.size() < 2 || !route_handler) {
    return std::nullopt;
  }*/
  static const auto logger_ =
  rclcpp::get_logger("direction_change");

  const auto goal_idx_opt = autoware::motion_utils::findNearestIndex(path.points, goal_pose.position);
  if (goal_idx_opt >= path.points.size()) {
    return std::nullopt;
  }
  const size_t goal_idx = goal_idx_opt;

  const auto arc_lengths = utils::calcPathArcLengthArray(path);
  const double goal_s = arc_lengths.at(goal_idx);

  const double d_goal =
    autoware::motion_utils::calcLateralOffset(path.points, goal_pose.position, false);
  /*if (std::isnan(d_goal) || std::abs(d_goal) < kMinLateralShiftM) {
    PathWithLaneId result = path;
    snapPathEndToGoal(&result, goal_pose);
    return result;
  }*/

  const double ref_yaw_at_goal = tf2::getYaw(path.points.at(goal_idx).point.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_pose.orientation);
  const double goal_yaw_delta = autoware_utils::normalize_radian(goal_yaw - ref_yaw_at_goal);

  const double max_allowed_yaw_rad = autoware_utils::deg2rad(parameters.max_allowed_yaw_deg);
  const double maneuver_length = computeManeuverLength(d_goal, max_allowed_yaw_rad);

  const double shift_start_s = goal_s - maneuver_length;
  std::cout << "[Debug applyGoalLateralShift] shift_start_s: " << shift_start_s 
            << ", maneuver_length: " << maneuver_length 
            << ", goal_s: " << goal_s << std::endl;

  if (shift_start_s < 0.0) {
    RCLCPP_WARN(
      logger_,
      "Infeasible lateral shift request. "
      "Required maneuver length: %.2f m, "
      "available distance: %.2f m",
      maneuver_length,
      goal_s);

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

bool hasDirectionChangeAreaTag(const lanelet::ConstLanelet & lanelet)
{
  const std::string direction_change_lane = lanelet.attributeOr("direction_change_lane", "none");
  return direction_change_lane == "yes";
}

bool checkLaneContinuitySafety(
  const PathWithLaneId & path, const std::vector<size_t> & cusp_indices,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  // TODO: Implementation placeholder for critical safety check
  //
  // Safety Check: Lane Continuity with Reverse Exit
  //
  // Problem Scenario:
  // - Odd number of cusps means vehicle exits current lane in REVERSE mode
  // - If next lane doesn't have turn_area tag, Autoware defaults to FORWARD following
  // - This creates a FATAL condition: vehicle reversing but system expects forward motion
  //
  // Algorithm:
  // 1. Count number of cusps in current lane (before lane boundary)
  // 2. If odd number of cusps:
  //    a. Find next lane (after lane boundary)
  //    b. Check if next lane has turn_area tag
  //    c. If next lane doesn't have tag → FATAL CONDITION (return false)
  //    d. If next lane has tag → Safe (can continue reverse, return true)
  // 3. If even number of cusps:
  //    a. Vehicle exits in FORWARD mode → Safe (return true)
  //
  // Implementation Steps:
  // - Use detectLaneBoundaries() to find lane transition points
  // - Count cusps before each lane boundary
  // - Check turn_area tag for next lane after boundary
  // - Return false if fatal condition detected, true otherwise
  //
  // For now, return true (assume safe) until full implementation
  // This should be implemented before production use

  if (!route_handler || path.points.empty() || cusp_indices.empty()) {
    return true;  // No cusps or no route handler, assume safe
  }

  return true;  // Placeholder: assume safe for now
}

void densifyPathByYawAndDistance(
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const double max_yaw_step_rad, const double max_dist_step)
{
  if (points.size() < 2) return;

  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> dense;
  dense.reserve(points.size() * 2);

  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & p0 = points[i].point;
    const auto & p1 = points[i + 1].point;

    // Always add point p0.
    dense.push_back(points[i]);

    const double x0 = p0.pose.position.x;
    const double y0 = p0.pose.position.y;
    const double z0 = p0.pose.position.z;
    const double x1 = p1.pose.position.x;
    const double y1 = p1.pose.position.y;
    const double z1 = p1.pose.position.z;

    const double dist = std::hypot(x1 - x0, y1 - y0);

    const double yaw0 = tf2::getYaw(p0.pose.orientation);
    const double yaw1 = tf2::getYaw(p1.pose.orientation);
    double dyaw = autoware_utils::normalize_radian(yaw1 - yaw0);

    // Calculate the number of segments
    int n_yaw = static_cast<int>(std::ceil(std::fabs(dyaw) / max_yaw_step_rad));
    int n_dist = static_cast<int>(std::ceil(dist / max_dist_step));
    int N = std::max(n_yaw, n_dist);

    if (N <= 1) {
      continue;  // Interpolation is not necessary.
    }

    // Insert N-1 intermediate points (0<k<N)
    for (int k = 1; k < N; ++k) {
      double r = static_cast<double>(k) / static_cast<double>(N);

      autoware_internal_planning_msgs::msg::PathPointWithLaneId mid;
      mid = points[i];  // Copy lane_ids etc.

      auto & mp = mid.point;
      mp.pose.position.x = x0 + (x1 - x0) * r;
      mp.pose.position.y = y0 + (y1 - y0) * r;
      mp.pose.position.z = z0 + (z1 - z0) * r;

      double yaw_mid = autoware_utils::normalize_radian(yaw0 + dyaw * r);
      mp.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw_mid);

      // Speed and other fields are not modified here (later DirectionChange will overwrite them)
      // Use lane_ids from p0 (mid.lane_ids = points[i].lane_ids;)

      dense.push_back(mid);
    }
  }

  // Don't forget to add the last point.
  dense.push_back(points.back());

  points.swap(dense);
}

}  // namespace autoware::behavior_path_planner
