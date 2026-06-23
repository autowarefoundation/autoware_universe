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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UPDATE__VEHICLE_UPDATE_STRATEGY_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UPDATE__VEHICLE_UPDATE_STRATEGY_HPP_

#include "autoware/multi_object_tracker/types.hpp"

#include <array>

namespace autoware::multi_object_tracker
{

enum class UpdateStrategyType { FRONT_WHEEL_UPDATE, REAR_WHEEL_UPDATE, WEAK_UPDATE };

struct UpdateStrategy
{
  UpdateStrategyType type;
  geometry_msgs::msg::Point anchor_point;  // used for FRONT_WHEEL_UPDATE and REAR_WHEEL_UPDATE
};

// Determines whether to use front-wheel, rear-wheel, or weak update
// by finding the closest edge pair between measurement and prediction.
UpdateStrategy determineUpdateStrategy(
  const types::DynamicObject & measurement, const types::DynamicObject & prediction);

// Blends measurement position/orientation into a copy of prediction using a distance-weighted
// scheme and returns it. When enlarge_covariance=true, inflates pose/velocity covariances for the
// weak-update path.
types::DynamicObject createPseudoMeasurement(
  const types::DynamicObject & meas, const types::DynamicObject & prediction,
  const bool enlarge_covariance = false);

// Scalar lateral correction of the wheel-anchor from the tracker and polygon left/right edges.
// tracker_left/right and polygon_left/right are signed body-frame lateral coordinates (positive =
// left). The
// per-side overhangs of the polygon beyond the tracker box give the slack (half the width excess)
// and the edge-center offset that drive a soft dead-zone ("back-lash") correction: the anchor is
// held near the tracker while contained in the polygon and follows an exposed corner once it
// protrudes. Returns the lateral move and, via `var_lat`, the extra lateral variance from the
// uncorrected residual.
double correctWheelAnchorLateral(
  const double tracker_left, const double tracker_right, const double polygon_left,
  const double polygon_right, double & var_lat);

// Laterally corrects the anchor against `prediction` and folds the extra variance into `pose_cov`.
geometry_msgs::msg::Point correctWheelAnchor(
  const types::DynamicObject & prediction, const double polygon_width,
  const geometry_msgs::msg::Point & anchor, std::array<double, 36> & pose_cov);

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UPDATE__VEHICLE_UPDATE_STRATEGY_HPP_
