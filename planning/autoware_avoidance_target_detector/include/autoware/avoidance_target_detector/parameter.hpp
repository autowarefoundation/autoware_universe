// Copyright 2026 Autoware Foundation
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

#ifndef AUTOWARE__AVOIDANCE_TARGET_DETECTOR__PARAMETER_HPP_
#define AUTOWARE__AVOIDANCE_TARGET_DETECTOR__PARAMETER_HPP_

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace autoware::avoidance_target_detector
{

using autoware_perception_msgs::msg::ObjectClassification;

/** Object classification labels considered as avoidance targets. */
inline constexpr std::array<uint8_t, 6> labels_of_interest{
  ObjectClassification::CAR,     ObjectClassification::TRUCK,      ObjectClassification::BUS,
  ObjectClassification::TRAILER, ObjectClassification::MOTORCYCLE, ObjectClassification::BICYCLE};

/** Parameters for filtering out moving objects by linear twist norm. */
struct MovingObjectFilterParams
{
  static constexpr double promising_stop_velocity_mps = 0.28;
  static constexpr double max_linear_velocity_mps =
    1.0;  ///< Objects with linear velocity norm above this are regarded as moving [m/s].
};

/** Parameters for on-trajectory d-coordinate validation (filter-out). */
struct OnTrajectoryDValidationParams
{
  static constexpr std::size_t sample_count_m = 5;      ///< Number of trajectory samples (M).
  static constexpr double magnitude_threshold_m = 1.5;  ///< Max |d(k)| threshold [m].
  static constexpr double deviation_threshold_m = 0.3;  ///< Max |d(k) - d(k-1)| threshold [m].
  static constexpr double near_s_range_m = 15.0;        ///< s-range around footprint (S) [m].
  static constexpr double s_sample_interval_m = 1.0;    ///< Fallback s sampling interval [m].
};

/** Parameters for longitudinal distance filtering using trajectory arc length. */
struct LongitudinalDistanceFilterParams
{
  double tolerance_m{0.0};  ///< Margin before start / after end [m].
};

/** Parameters for lateral distance filtering using drivable area bounds. */
struct LateralDistanceFilterParams
{
  double tolerance_m{0.0};  ///< Margin outside left/right bounds [m].
};

/** Parameters controlling AvoidanceTargetDetector target-state hysteresis and staleness. */
struct FilterManagerParams
{
  static constexpr double stale_threshold_seconds = 1.0;
  static constexpr double static_hysteresis_seconds = 0.5;
  static constexpr double moving_hysteresis_seconds = 5.0;
  static constexpr uint8_t count_threshold = 3;
};

}  // namespace autoware::avoidance_target_detector

#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__PARAMETER_HPP_
