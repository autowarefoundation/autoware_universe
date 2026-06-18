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

// Blends measurement position/orientation into pred using a distance-weighted scheme.
// When enlarge_covariance=true, inflates pose/velocity covariances for the weak-update path.
void createPseudoMeasurement(
  const types::DynamicObject & meas, types::DynamicObject & pred,
  const autoware_perception_msgs::msg::Shape & tracker_shape,
  const bool enlarge_covariance = false);

// Result of the wheel-anchor lateral correction.
struct WheelAnchorLateral
{
  geometry_msgs::msg::Point anchor;  // laterally corrected anchor point
  double var_lat;                    // extra variance to add along the body lateral axis [m^2]
};

// Corrects the wheel-anchor lateral position and reports extra lateral variance, accounting for the
// mismatch between the observed polygon width and the tracked width. The wheel update measures the
// observed front/rear edge CENTER, which is a biased lateral measurement when the widths differ:
//   - polygon NARROWER (partial view): the center may sit anywhere within (w_t - w_p)/2 of the true
//     center. The anchor is kept and the worst-case lateral offset is added as variance.
//   - polygon WIDER (merged / over-segmented cluster): the edge center is pulled off the true body.
//     A soft dead-zone ("back-lash") of half-width s = (w_p - w_t)/2 is applied to the lateral
//     offset d between the observed edge center and the tracker center: the anchor is held near the
//     tracker (slope `balance_alpha`) while the tracker stays contained (|d| <= s) and follows the
//     exposed corner (unit slope) once |d| > s. The added lateral std scales continuously from s
//     (centered, true position unknown across the slack) down to `corner_residual_beta` * s once
//     the corner is matched.
// `tracker_center` is the tracked body center (e.g. the prediction pose); `anchor` is the observed
// edge center. The correction only moves the lateral component, leaving the longitudinal anchor
// (and hence the length estimate) untouched.
WheelAnchorLateral correctWheelAnchorLateral(
  double yaw, double tracker_width, const geometry_msgs::msg::Point & tracker_center,
  double polygon_width, const geometry_msgs::msg::Point & anchor, double balance_alpha,
  double corner_residual_beta);

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UPDATE__VEHICLE_UPDATE_STRATEGY_HPP_
