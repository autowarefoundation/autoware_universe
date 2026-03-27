// Copyright 2025 Tier IV, Inc.
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

#include "autoware/multi_object_tracker/tracker/shape_model/unstable_shape_filter.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <cmath>

namespace autoware::multi_object_tracker
{

UnstableShapeFilter::UnstableShapeFilter(
  double alpha_weak, double alpha_strong, double shape_variation_threshold,
  int stable_streak_threshold, int consecutive_noisy_threshold)
: initialized_(false),
  stable_(false),
  alpha_weak_(alpha_weak),
  alpha_strong_(alpha_strong),
  shape_variation_threshold_(shape_variation_threshold),
  stable_streak_(0),
  stable_streak_threshold_(stable_streak_threshold),
  consecutive_noisy_frames_(0),
  consecutive_noisy_threshold_(consecutive_noisy_threshold),
  normal_frame_interruptions_(0)
{
  // Initialize latest_shape_ with safe defaults
  latest_shape_.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  latest_shape_.dimensions.x = 0.0;
  latest_shape_.dimensions.y = 0.0;
  latest_shape_.dimensions.z = 0.0;
}

void UnstableShapeFilter::initialize(const Eigen::Vector3d & initial_shape)
{
  value_ = initial_shape;
  initialized_ = true;
  stable_ = false;
  stable_streak_ = 0;
}

void UnstableShapeFilter::clear()
{
  initialized_ = false;
  stable_ = false;
  stable_streak_ = 0;
  consecutive_noisy_frames_ = 0;
  normal_frame_interruptions_ = 0;
}

void UnstableShapeFilter::processNoisyMeasurement(const types::DynamicObject & measurement)
{
  // Apply exponential moving average (EMA) smoothing for BOUNDING_BOX
  if (measurement.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    stable_ = false;
    latest_shape_ = measurement.shape;
    return;
  }

  // Get shape dimensions
  const Eigen::Vector3d meas{
    measurement.shape.dimensions.x, measurement.shape.dimensions.y, measurement.shape.dimensions.z};

  // Initialize the filter if not already done
  if (!initialized_) {
    initialize(meas);
    return;
  }

  // Track consecutive noisy measurements - builds confidence in new shape
  ++consecutive_noisy_frames_;
  normal_frame_interruptions_ = 0;  // Reset normal frame counter

  // Update shape dimensions using dual-rate EMA
  Eigen::Vector3d rel = (meas - value_).cwiseAbs().cwiseQuotient(value_.cwiseMax(1e-3));
  if (rel.maxCoeff() < shape_variation_threshold_) {
    // Noisy measurement is close to current value_ - shape is converging
    value_ = alpha_strong_ * meas + (1.0 - alpha_strong_) * value_;
    ++stable_streak_;

    // Require both: stable_streak AND sufficient consecutive noisy frames
    // This ensures the new shape has been consistently observed
    if (
      stable_streak_ >= stable_streak_threshold_ &&
      consecutive_noisy_frames_ >= consecutive_noisy_threshold_) {
      stable_ = true;
    }
  } else {
    // Noisy measurement differs from value_ - shape still changing
    stable_streak_ = 0;
    stable_ = false;
    // Use weaker update when shape is still varying
    value_ = alpha_weak_ * meas + (1.0 - alpha_weak_) * value_;
  }
}

void UnstableShapeFilter::processNormalMeasurement(const types::DynamicObject & measurement)
{
  latest_shape_ = measurement.shape;

  // Normal measurement interrupts the noisy sequence
  ++normal_frame_interruptions_;

  // Reset the filter if it was likely tracking a temporary noisy shape
  if (normal_frame_interruptions_ >= 2) {
    consecutive_noisy_frames_ = 0;
    stable_ = false;
  }
}

autoware_perception_msgs::msg::Shape UnstableShapeFilter::getShape() const
{
  // if not stable, return the latest shape as-is (no smoothing)
  if (!stable_) return latest_shape_;

  // For BOUNDING_BOX type, return the smoothed shape - value_
  autoware_perception_msgs::msg::Shape shape;
  shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;

  shape.dimensions.x = value_(0);  // length
  shape.dimensions.y = value_(1);  // width
  shape.dimensions.z = value_(2);  // height

  return shape;
}

}  // namespace autoware::multi_object_tracker
