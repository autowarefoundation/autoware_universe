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

#include "autoware/multi_object_tracker/association/scoring/polar_assignment_scoring.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::multi_object_tracker
{
namespace polar_scoring
{

namespace
{
constexpr double MIN_RANGE = 1.0;  // Minimum range to avoid azimuth instability [m]
constexpr double MIN_SPAN = 1e-6;  // Minimum span to avoid division by zero

// Returns the azimuth intersection span [rad] between two azimuth intervals.
double azimuthIntersectionSpan(const AzimuthInterval & a, const AzimuthInterval & b)
{
  const double d = std::abs(normalizeAngle(a.center - b.center));
  const double left = std::max(-a.half_span, d - b.half_span);
  const double right = std::min(a.half_span, d + b.half_span);
  return std::max(0.0, right - left);
}

// Returns the minimum 3D Euclidean distance from ego to any corner of the object's bounding box.
// Iterates all (2D polygon corners) × (z_min, z_max) to cover all 3D box corners.
// Accepts pre-computed polygon and z-range to avoid redundant to_polygon2d / getObjectZRange calls.
double computeClosestCorner3d(
  const types::DynamicObject & object, const autoware_utils_geometry::Polygon2d & polygon,
  const double z_min, const double z_max, const double ego_x, const double ego_y,
  const double ego_z)
{
  const auto & pts = polygon.outer();

  if (pts.size() < 2) {
    const double dx = object.pose.position.x - ego_x;
    const double dy = object.pose.position.y - ego_y;
    const double dz = object.pose.position.z - ego_z;
    return std::max(MIN_RANGE, std::sqrt(dx * dx + dy * dy + dz * dz));
  }

  double r_min_3d = std::numeric_limits<double>::max();
  for (const auto & pt : pts) {
    const double dx = pt.x() - ego_x;
    const double dy = pt.y() - ego_y;
    for (const double cz : {z_min, z_max}) {
      const double dz = cz - ego_z;
      r_min_3d = std::min(r_min_3d, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
  }
  return std::max(r_min_3d, MIN_RANGE);
}
}  // namespace

PolarFootprint computePolarFootprint(
  const types::DynamicObject & object, const double ego_x, const double ego_y, const double ego_z,
  const double ego_yaw)
{
  // Get the 2D polygon corners in map frame
  const auto polygon = autoware_utils_geometry::to_polygon2d(object.pose, object.shape);
  const auto & points = polygon.outer();

  // Height range from object shape
  const auto [z_min, z_max] = shapes::getObjectZRange(object);

  const double r_min_3d =
    computeClosestCorner3d(object, polygon, z_min, z_max, ego_x, ego_y, ego_z);

  if (points.size() < 2) {
    // Degenerate polygon: use object center as a single point
    const double dx = object.pose.position.x - ego_x;
    const double dy = object.pose.position.y - ego_y;
    const double range = std::max(MIN_RANGE, std::sqrt(dx * dx + dy * dy));
    const double azimuth = normalizeAngle(std::atan2(dy, dx) - ego_yaw);
    return {{azimuth, 0.0}, range, range, r_min_3d, z_min, z_max};
  }

  // Centroid direction used as the reference from which corner offsets are measured.
  // For partial observations (I-shape, L-shape) the centroid may not be the angular
  // midpoint of the visible polygon, so we recompute the true angular midpoint below.
  const double cx = object.pose.position.x - ego_x;
  const double cy = object.pose.position.y - ego_y;
  const double center_azimuth = normalizeAngle(std::atan2(cy, cx) - ego_yaw);

  double r_min = std::numeric_limits<double>::max();
  double r_max = 0.0;
  // Signed angular offsets from center_azimuth tracked independently.
  double az_lo = std::numeric_limits<double>::max();
  double az_hi = std::numeric_limits<double>::lowest();

  for (const auto & pt : points) {
    const double dx = pt.x() - ego_x;
    const double dy = pt.y() - ego_y;
    const double range = std::sqrt(dx * dx + dy * dy);
    r_min = std::min(r_min, range);
    r_max = std::max(r_max, range);

    // Skip only truly degenerate corners (corner at ego position).
    if (range < 1e-6) continue;

    const double azimuth = normalizeAngle(std::atan2(dy, dx) - ego_yaw);
    const double offset = normalizeAngle(azimuth - center_azimuth);  // signed
    az_lo = std::min(az_lo, offset);
    az_hi = std::max(az_hi, offset);
  }

  r_min = std::max(r_min, MIN_RANGE);

  if (az_lo > az_hi) {
    // All corners were at the ego position — degenerate polygon.
    return {{center_azimuth, 0.0}, r_min, r_max, r_min_3d, z_min, z_max};
  }

  // Angular midpoint of the visible span, corrected from the centroid's azimuth.
  const double corrected_center = normalizeAngle(center_azimuth + (az_lo + az_hi) / 2.0);
  const double half_span = (az_hi - az_lo) / 2.0;

  return {{corrected_center, half_span}, r_min, r_max, r_min_3d, z_min, z_max};
}

double azimuthIoU(const AzimuthInterval & a, const AzimuthInterval & b)
{
  const double intersection = azimuthIntersectionSpan(a, b);
  const double union_span = 2.0 * a.half_span + 2.0 * b.half_span - intersection;
  if (union_span < MIN_SPAN) return 0.0;
  return std::min(1.0, intersection / union_span);
}

ScoringResult calculatePolarAssignmentScore(
  const PolarFootprint & meas_fp, const PolarFootprint & tracker_fp,
  const types::DynamicObject & measurement_object, const types::DynamicObject & tracked_object,
  const types::TrackerType tracker_type, const double min_iou)
{
  ScoringResult result{0.0, false};

  // 2D perspective IoU in (azimuth [rad] × height [m]) space.
  const double az_inter = azimuthIntersectionSpan(meas_fp.azimuth, tracker_fp.azimuth);
  const double h_inter = std::max(
    0.0, std::min(meas_fp.z_max, tracker_fp.z_max) - std::max(meas_fp.z_min, tracker_fp.z_min));

  const double inter_area = az_inter * h_inter;
  const double area_meas_fp = 2.0 * meas_fp.azimuth.half_span * (meas_fp.z_max - meas_fp.z_min);
  const double area_trk_fp =
    2.0 * tracker_fp.azimuth.half_span * (tracker_fp.z_max - tracker_fp.z_min);
  const double union_area = area_meas_fp + area_trk_fp - inter_area;

  const double perspective_iou = (union_area > MIN_SPAN) ? inter_area / union_area : 0.0;

  if (perspective_iou < min_iou) return result;

  const double iou_score = (perspective_iou - min_iou) / (1.0 - min_iou);

  const bool is_vehicle = isVehicleTrackerType(tracker_type);

  // Shape change detection for vehicle trackers with trusted shape measurements
  if (
    perspective_iou < PERSPECTIVE_IOU_SHAPE_CHECK_THRESHOLD && is_vehicle &&
    measurement_object.trust_extension) {
    const double area_meas = measurement_object.area;
    const double area_trk = tracked_object.area;
    if (area_meas > 0.0 && area_trk > 0.0) {
      const double area_ratio = std::max(area_trk, area_meas) / std::min(area_trk, area_meas);
      if (area_ratio > AREA_RATIO_THRESHOLD) {
        result.has_significant_shape_change = true;
      }
    }
  }

  // Blend IoU score with graded closest-point proximity.
  // The depth proximity term rewards measurements whose nearest visible surface closely matches
  // the tracker's nearest surface — the primary LiDAR observable for partial returns.
  // Vehicles use a higher weight (0.15) because partial-face returns make depth the dominant
  // physical cue; non-vehicles use a lighter tie-breaker (0.05).
  const double depth_diff = std::abs(meas_fp.r_min_3d - tracker_fp.r_min_3d);
  const double depth_proximity = std::max(0.0, 1.0 - depth_diff / DEPTH_PROXIMITY_SCALE);
  if (is_vehicle) {
    result.score =
      iou_score * (1.0 - DEPTH_PROXIMITY_WEIGHT) + depth_proximity * DEPTH_PROXIMITY_WEIGHT;
  } else {
    result.score = iou_score * (1.0 - DEPTH_PROXIMITY_WEIGHT_NON_VEHICLE) +
                   depth_proximity * DEPTH_PROXIMITY_WEIGHT_NON_VEHICLE;
  }

  return result;
}

}  // namespace polar_scoring
}  // namespace autoware::multi_object_tracker
