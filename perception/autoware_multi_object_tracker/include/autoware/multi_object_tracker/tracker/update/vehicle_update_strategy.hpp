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

// Result of the wheel-anchor lateral correction.
struct WheelAnchorLateral
{
  geometry_msgs::msg::Point anchor;  // laterally corrected anchor point
  double var_lat;                    // extra variance to add along the body lateral axis [m^2]
};

// Laterally corrects the wheel-anchor and reports extra lateral variance to handle the bias that
// arises when the observed front/rear edge center is used as a lateral measurement but the polygon
// and tracked widths disagree. Only the lateral component is affected.
WheelAnchorLateral correctWheelAnchorLateral(
  double yaw, double tracker_width, const geometry_msgs::msg::Point & tracker_center,
  double polygon_width, const geometry_msgs::msg::Point & anchor, double balance_alpha,
  double corner_residual_beta);

// Applies the wheel-anchor lateral correction (see correctWheelAnchorLateral) and folds the
// resulting extra lateral variance into the x/y block of `pose_cov`. Returns the corrected anchor.
geometry_msgs::msg::Point correctWheelAnchor(
  double yaw, double tracker_width, const geometry_msgs::msg::Point & tracker_center,
  double polygon_width, const geometry_msgs::msg::Point & anchor,
  std::array<double, 36> & pose_cov);

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UPDATE__VEHICLE_UPDATE_STRATEGY_HPP_
