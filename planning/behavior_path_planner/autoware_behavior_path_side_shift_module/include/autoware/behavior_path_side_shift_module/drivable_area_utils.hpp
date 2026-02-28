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

#ifndef AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__DRIVABLE_AREA_UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__DRIVABLE_AREA_UTILS_HPP_

#include <lanelet2_core/primitives/Lanelet.h>

namespace autoware::behavior_path_planner
{

// Small additional safety buffer [m] applied on top of min_drivable_area_margin.
// lanelet::geometry::distance2d() computes distance to the nearest polyline segment of a lanelet
// boundary. Because boundaries are piecewise-linear approximations of the real road edge, the
// true perpendicular distance can be shorter than the computed value by up to:
//   error <= segment_length² / (8 * curvature_radius)
// For typical Autoware lanelet maps (segment ≈ 1 m, curvature radius >= 10 m) this is ≈ 0.013 m.
// We use 0.05 m (~4x worst-case) to also absorb floating-point rounding in coordinate transforms
// and interpolation. This is intentionally NOT a tunable parameter: it is a numerical guard rather
// than a design-level margin (use min_drivable_area_margin for that purpose).
constexpr double kLaneBoundaryDiscretizationBuffer = 0.05;

/// Information about the adjacent lanes of a given lanelet.
struct AdjacentLaneInfo
{
  bool allow_left{false};
  bool allow_right{false};
  bool has_left{false};
  bool has_right{false};
  lanelet::ConstLanelet left_lane;
  lanelet::ConstLanelet right_lane;

  /// True when shifting into the left adjacent lane is both allowed and the lane exists.
  bool canShiftLeft() const { return allow_left && has_left; }
  /// True when shifting into the right adjacent lane is both allowed and the lane exists.
  bool canShiftRight() const { return allow_right && has_right; }
};

/// Accumulated lane boundary limits used by the drivable-area check.
struct LaneLimitInfo
{
  double safe_left_limit{100.0};
  double safe_right_limit{100.0};
  bool found_valid_limit{false};
};

/// Result of determining which lane a point belongs to.
struct LanePositionResult
{
  lanelet::ConstLanelet check_lane;
  bool is_in_adjacent{false};
  bool is_outside_all{false};
};

/// Context bundling lane position, adjacency info, and vehicle geometry for drivable-area checks.
struct LaneCheckContext
{
  LanePositionResult lane_position;
  AdjacentLaneInfo adjacent_info;
  double vehicle_half_width{0.0};
  double margin{0.0};
};

/**
 * @brief Determine which lane a point belongs to (current, adjacent, or outside all).
 * @param lane          The current (closest-route) lanelet.
 * @param target_point  2-D point to classify.
 * @param adj_info      Adjacent lane information.
 * @return A LanePositionResult indicating the lane and classification.
 */
LanePositionResult determineLanePosition(
  const lanelet::ConstLanelet & lane, const lanelet::BasicPoint2d & target_point,
  const AdjacentLaneInfo & adj_info);

/**
 * @brief Update lane limits when a reference point is outside all known lanes.
 *
 * Only the side closest to the point is penalised (limit set to 0), while the
 * opposite side gets just enough room to "return" into the lane.
 *
 * @param target_point       The reference point.
 * @param ctx                Lane check context (position, adjacency, vehicle geometry).
 * @param[in,out] limits     Accumulated lane limits (modified in place).
 */
void updateLaneLimitsForOutsidePoint(
  const lanelet::BasicPoint2d & target_point, const LaneCheckContext & ctx, LaneLimitInfo & limits);

/**
 * @brief Update lane limits when a reference point is inside a lane.
 *
 * Computes the maximum left/right shift that keeps the vehicle footprint
 * (half-width + margin + discretization buffer) inside the lane boundaries
 * (or extended into an adjacent lane in mode 2).
 *
 * @param target_point       The reference point.
 * @param ctx                Lane check context (position, adjacency, vehicle geometry).
 * @param[in,out] limits     Accumulated lane limits (modified in place).
 */
void updateLaneLimitsForInsidePoint(
  const lanelet::BasicPoint2d & target_point, const LaneCheckContext & ctx, LaneLimitInfo & limits);

/**
 * @brief Clamp a requested lateral offset to the computed safe limits.
 *
 * Positive offsets (left) are clamped to @p safe_left_limit; negative offsets
 * (right) are clamped to -@p safe_right_limit. Values whose absolute magnitude
 * falls below 1 mm are snapped to zero.
 *
 * @param requested_offset  The desired lateral offset [m].
 * @param safe_left_limit   Maximum allowable leftward shift [m] (>= 0).
 * @param safe_right_limit  Maximum allowable rightward shift [m] (>= 0).
 * @return The clamped offset.
 */
double clampOffsetToLimits(
  double requested_offset, double safe_left_limit, double safe_right_limit);

/**
 * @brief Prevent the clamped offset from jumping backward past the current base.
 *
 * When the user requests *more* shift in a given direction but the boundary
 * check returns a value smaller than the current offset (due to tighter limits
 * at future reference points), this function floors / ceils the result at
 * @p current_base so the vehicle never snaps back.
 *
 * @param requested_offset  The original user request [m].
 * @param clamped_offset    The offset already clamped to lane limits [m].
 * @param current_base      The current PathShifter base offset [m].
 * @return The guarded offset.
 */
double guardAgainstSnapBack(double requested_offset, double clamped_offset, double current_base);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__DRIVABLE_AREA_UTILS_HPP_
