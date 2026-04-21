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

#include "autoware/multi_object_tracker/association/scoring/bev_assignment_scoring.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::multi_object_tracker
{
namespace
{
constexpr double INVALID_SCORE = 0.0;

inline double getMahalanobisDistanceFast(double dx, double dy, const InverseCovariance2D & inv_cov)
{
  return dx * dx * inv_cov.inv00 + 2.0 * dx * dy * inv_cov.inv01 + dy * dy * inv_cov.inv11;
}
}  // namespace

double calculateBevAssignmentScore(
  const types::DynamicObject & tracked_object, const classes::Label tracker_label,
  const types::TrackerType tracker_type,
  const AssociatorConfig::TrackerAssociationParameters & association_params,
  const types::DynamicObject & measurement_object, const classes::Label measurement_label,
  const InverseCovariance2D & inv_cov, const double unknown_association_giou_threshold,
  bool & has_significant_shape_change)
{
  // When both tracker and measurement are unknown, use generalized IoU only
  if (tracker_label == classes::Label::UNKNOWN && measurement_label == classes::Label::UNKNOWN) {
    const double generalized_iou = shapes::get2dGeneralizedIoU(tracked_object, measurement_object);
    if (generalized_iou < unknown_association_giou_threshold) {
      return INVALID_SCORE;
    }
    // Rescale score to [0, 1]
    return (generalized_iou - unknown_association_giou_threshold) /
           (1.0 - unknown_association_giou_threshold);
  }

  const double max_dist_sq = association_params.max_dist_sq;
  const double dx = measurement_object.pose.position.x - tracked_object.pose.position.x;
  const double dy = measurement_object.pose.position.y - tracked_object.pose.position.y;
  const double dist_sq = dx * dx + dy * dy;

  // Distance gate
  if (dist_sq > max_dist_sq) return INVALID_SCORE;

  // Gates for non-vehicle objects
  const double area_meas = measurement_object.area;
  const bool is_vehicle_tracker = isVehicleTrackerType(tracker_type);
  if (!is_vehicle_tracker) {
    // Area gate
    const double max_area = association_params.max_area;
    const double min_area = association_params.min_area;
    if (area_meas < min_area || area_meas > max_area) return INVALID_SCORE;

    // Mahalanobis distance gate
    const double mahalanobis_dist = getMahalanobisDistanceFast(dx, dy, inv_cov);
    // Empirical value: 99.6% confidence level for chi-square with 2 DOF
    constexpr double mahalanobis_dist_threshold = 11.62;
    if (mahalanobis_dist >= mahalanobis_dist_threshold) return INVALID_SCORE;
  }

  const double min_iou = association_params.min_iou;

  // Use 1D IoU for pedestrian, 3D GIoU if both extension values are trustable, else 2D GIoU
  const bool use_1d_iou = (tracker_label == classes::Label::PEDESTRIAN);
  const bool use_3d_iou = tracked_object.trust_extension && measurement_object.trust_extension;

  double iou_score;
  if (use_1d_iou) {
    iou_score = shapes::get1dIoU(measurement_object, tracked_object);
  } else if (use_3d_iou) {
    iou_score = shapes::get3dGeneralizedIoU(measurement_object, tracked_object);
  } else {
    iou_score = shapes::get2dGeneralizedIoU(measurement_object, tracked_object);
  }
  if (iou_score < min_iou) return INVALID_SCORE;

  // Check if shape changes too much for vehicle labels
  if (iou_score < CHECK_GIOU_THRESHOLD && is_vehicle_tracker) {
    const double area_trk = tracked_object.area;
    const double area_ratio = std::max(area_trk, area_meas) / std::min(area_trk, area_meas);
    if (area_ratio > AREA_RATIO_THRESHOLD) {
      has_significant_shape_change = true;
    }
  }

  // Rescale score to [0, 1]
  return (iou_score - min_iou) / (1.0 - min_iou);
}

}  // namespace autoware::multi_object_tracker
