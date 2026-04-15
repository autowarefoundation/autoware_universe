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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__SCORING__BEV_ASSIGNMENT_SCORING_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__SCORING__BEV_ASSIGNMENT_SCORING_HPP_

#include "autoware/multi_object_tracker/configurations.hpp"
#include "autoware/multi_object_tracker/types.hpp"

namespace autoware::multi_object_tracker
{

// Precomputed 2×2 inverse covariance for fast Mahalanobis distance calculation
struct InverseCovariance2D
{
  double inv00;  // (d / det)
  double inv01;  // (-b / det)
  double inv11;  // (a / det)
};

// GIoU threshold below which the shape-change check is activated for vehicle trackers
constexpr double CHECK_GIOU_THRESHOLD = 0.7;
// Area ratio above which the shape change is considered significant
constexpr double AREA_RATIO_THRESHOLD = 1.3;

/// Computes a [0, 1] assignment score between a tracker and a measurement for D2T GNN assignment.
/// Returns 0.0 (INVALID_SCORE) when the pair fails any gate and should be rejected.
/// Sets has_significant_shape_change=true when matched vehicle objects differ noticeably in size.
double calculateBevAssignmentScore(
  const types::DynamicObject & tracked_object, const classes::Label tracker_label,
  const types::TrackerType tracker_type,
  const AssociatorConfig::TrackerAssociationParameters & association_params,
  const types::DynamicObject & measurement_object, const classes::Label measurement_label,
  const InverseCovariance2D & inv_cov, double unknown_association_giou_threshold,
  bool & has_significant_shape_change);

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__SCORING__BEV_ASSIGNMENT_SCORING_HPP_
