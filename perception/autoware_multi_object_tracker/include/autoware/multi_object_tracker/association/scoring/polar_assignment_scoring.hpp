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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__SCORING__POLAR_ASSIGNMENT_SCORING_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__SCORING__POLAR_ASSIGNMENT_SCORING_HPP_

#include "autoware/multi_object_tracker/association/scoring/scoring_types.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <cmath>

namespace autoware::multi_object_tracker
{
namespace polar_scoring
{

/// Azimuth interval in center + half-span form.
/// This representation handles angle wrapping naturally: the angular distance
/// between two centers is always computed via normalizeAngle(), so intervals
/// near the +/-pi boundary compare correctly without explicit wrapping logic.
struct AzimuthInterval
{
  double center;     // center angle [rad], in [-pi, pi]
  double half_span;  // half angular width [rad], >= 0
};

/// Polar footprint of an object as seen from the ego vehicle.
struct PolarFootprint
{
  AzimuthInterval azimuth;
  double r_min;     // minimum 2D XY range from ego [m]
  double r_max;     // maximum 2D XY range from ego [m]
  double r_min_3d;  // minimum 3D Euclidean distance to nearest box corner [m]
  double z_min;     // minimum height [m]
  double z_max;     // maximum height [m]
};

/// Normalize angle to (-pi, pi].
inline double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

// Perspective IoU threshold below which the shape-change check is activated for vehicle trackers
constexpr double PERSPECTIVE_IOU_SHAPE_CHECK_THRESHOLD = 0.7;
// Area ratio above which the shape change is considered significant
constexpr double AREA_RATIO_THRESHOLD = 2.0;
// Scale [m] for graded closest-point proximity in vehicle tracker scoring
constexpr double DEPTH_PROXIMITY_SCALE = 3.0;
// Weight of closest-point proximity in the combined score for vehicle trackers
constexpr double DEPTH_PROXIMITY_WEIGHT = 0.02;

/// Compute polar footprint of an object relative to ego position and heading.
/// Object pose and shape are in the map frame; the result is in ego-centric polar coordinates.
/// @param object  Dynamic object with pose and shape in map frame
/// @param ego_x   Ego position x in map frame [m]
/// @param ego_y   Ego position y in map frame [m]
/// @param ego_z   Ego position z in map frame [m]
/// @param ego_yaw Ego heading in map frame [rad]
PolarFootprint computePolarFootprint(
  const types::DynamicObject & object, double ego_x, double ego_y, double ego_z, double ego_yaw);

/// 1D IoU of two azimuth intervals.
/// Uses the center+half_span representation to handle angle wrapping naturally.
/// @return value in [0, 1]
double azimuthIoU(const AzimuthInterval & a, const AzimuthInterval & b);

/// Compute a combined [0, 1] polar assignment score.
/// Uses 2D perspective IoU in (azimuth [rad] × height [m]) space, blended with a graded
/// closest-point proximity term for vehicle trackers.
/// Returns score=0.0 when the pair fails the min_iou gate.
ScoringResult calculatePolarAssignmentScore(
  const PolarFootprint & meas_fp, const PolarFootprint & tracker_fp,
  const types::DynamicObject & measurement_object, const types::DynamicObject & tracked_object,
  types::TrackerType tracker_type, double min_iou);

}  // namespace polar_scoring
}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__SCORING__POLAR_ASSIGNMENT_SCORING_HPP_
