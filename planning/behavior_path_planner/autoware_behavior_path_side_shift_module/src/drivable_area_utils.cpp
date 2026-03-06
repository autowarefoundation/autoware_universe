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

#include "autoware/behavior_path_side_shift_module/drivable_area_utils.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>

namespace autoware::behavior_path_planner
{

LanePositionResult determineLanePosition(
  const lanelet::ConstLanelet & lane, const lanelet::BasicPoint2d & target_point,
  const AdjacentLaneInfo & adj_info)
{
  LanePositionResult result;
  result.check_lane = lane;

  const bool in_current_lane = lanelet::geometry::inside(lane, target_point);
  if (in_current_lane) {
    return result;
  }

  // Not in current lane - check adjacent lanes
  const bool in_left_lane =
    adj_info.has_left && lanelet::geometry::inside(adj_info.left_lane, target_point);
  const bool in_right_lane =
    adj_info.has_right && lanelet::geometry::inside(adj_info.right_lane, target_point);

  if (in_left_lane) {
    result.check_lane = adj_info.left_lane;
    result.is_in_adjacent = true;
  } else if (in_right_lane) {
    result.check_lane = adj_info.right_lane;
    result.is_in_adjacent = true;
  } else {
    result.is_outside_all = true;
  }

  return result;
}

void updateLaneLimitsForOutsidePoint(
  const lanelet::BasicPoint2d & target_point, const LaneCheckContext & ctx, LaneLimitInfo & limits)
{
  const auto & lane = ctx.lane_position.check_lane;
  const auto left_bound = lane.leftBound2d();
  const auto right_bound = lane.rightBound2d();
  const double dist_to_left = lanelet::geometry::distance2d(left_bound, target_point);
  const double dist_to_right = lanelet::geometry::distance2d(right_bound, target_point);

  if (dist_to_left < dist_to_right) {
    // Point is on the left side - only allow shift back to the right
    limits.safe_left_limit = 0.0;
    const double return_distance = dist_to_left + ctx.vehicle_half_width + ctx.margin;
    limits.safe_right_limit = std::min(limits.safe_right_limit, return_distance);
  } else {
    // Point is on the right side - only allow shift back to the left
    limits.safe_right_limit = 0.0;
    const double return_distance = dist_to_right + ctx.vehicle_half_width + ctx.margin;
    limits.safe_left_limit = std::min(limits.safe_left_limit, return_distance);
  }
  limits.found_valid_limit = true;
}

void updateLaneLimitsForInsidePoint(
  const lanelet::BasicPoint2d & target_point, const LaneCheckContext & ctx, LaneLimitInfo & limits)
{
  const auto & lane = ctx.lane_position.check_lane;
  const auto & adj = ctx.adjacent_info;
  const bool point_in_adjacent = ctx.lane_position.is_in_adjacent;

  const auto left_bound = lane.leftBound2d();
  const auto right_bound = lane.rightBound2d();

  const double dist_to_left = lanelet::geometry::distance2d(left_bound, target_point);
  const double dist_to_right = lanelet::geometry::distance2d(right_bound, target_point);

  // Calculate maximum left shift (positive value)
  double current_max_left =
    dist_to_left - ctx.vehicle_half_width - ctx.margin - kLaneBoundaryDiscretizationBuffer;
  if (adj.canShiftLeft() && !point_in_adjacent) {
    const double dist_to_far_left =
      lanelet::geometry::distance2d(adj.left_lane.leftBound2d(), target_point);
    current_max_left =
      dist_to_far_left - ctx.vehicle_half_width - ctx.margin - kLaneBoundaryDiscretizationBuffer;
  }

  // Calculate maximum right shift
  double current_max_right =
    dist_to_right - ctx.vehicle_half_width - ctx.margin - kLaneBoundaryDiscretizationBuffer;
  if (adj.canShiftRight() && !point_in_adjacent) {
    const double dist_to_far_right =
      lanelet::geometry::distance2d(adj.right_lane.rightBound2d(), target_point);
    current_max_right =
      dist_to_far_right - ctx.vehicle_half_width - ctx.margin - kLaneBoundaryDiscretizationBuffer;
  }

  limits.safe_left_limit = std::min(limits.safe_left_limit, std::max(0.0, current_max_left));
  limits.safe_right_limit = std::min(limits.safe_right_limit, std::max(0.0, current_max_right));
  limits.found_valid_limit = true;
}

double clampOffsetToLimits(double requested_offset, double safe_left_limit, double safe_right_limit)
{
  constexpr double tolerance = 1e-3;

  if (requested_offset > 0.0) {
    // Left shift: clamp to safe_left_limit
    const double result = std::min(requested_offset, safe_left_limit);
    return (result < tolerance) ? 0.0 : result;
  } else if (requested_offset < 0.0) {
    // Right shift: clamp to negative of safe_right_limit
    const double result = std::max(requested_offset, -safe_right_limit);
    return (result > -tolerance) ? 0.0 : result;
  }
  return 0.0;
}

double guardAgainstSnapBack(
  const double requested_offset, const double clamped_offset, const double current_base)
{
  // User requests more leftward shift but clamp pulled it below current position
  if (requested_offset > current_base && clamped_offset < current_base) {
    return current_base;
  }
  // User requests more rightward shift but clamp pushed it above current position
  if (requested_offset < current_base && clamped_offset > current_base) {
    return current_base;
  }
  return clamped_offset;
}

}  // namespace autoware::behavior_path_planner
