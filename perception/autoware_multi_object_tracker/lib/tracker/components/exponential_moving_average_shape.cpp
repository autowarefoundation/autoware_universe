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

#include "autoware/multi_object_tracker/tracker/components/exponential_moving_average_shape.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <cmath>

namespace autoware::multi_object_tracker
{

ExponentialMovingAverageShape::ExponentialMovingAverageShape(
  double alpha_weak, double alpha_strong, double shape_variation_threshold,
  int stable_streak_threshold)
: initialized_(false),
  stable_(false),
  alpha_weak_(alpha_weak),
  alpha_strong_(alpha_strong),
  shape_variation_threshold_(shape_variation_threshold),
  stable_streak_(0),
  stable_streak_threshold_(stable_streak_threshold)
{
  // Initialize latest_shape_ with safe defaults
  latest_shape_.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  latest_shape_.dimensions.x = 0.0;
  latest_shape_.dimensions.y = 0.0;
  latest_shape_.dimensions.z = 0.0;
}

void ExponentialMovingAverageShape::initialize(const Eigen::Vector3d & initial_shape)
{
  value_ = initial_shape;
  initialized_ = true;
  stable_ = false;
  stable_streak_ = 0;
}

void ExponentialMovingAverageShape::clear()
{
  initialized_ = false;
  stable_ = false;
  stable_streak_ = 0;
}

void ExponentialMovingAverageShape::processNoisyMeasurement(
  const types::DynamicObject & measurement)
{
  // Store the latest shape
  latest_shape_ = measurement.shape;

  // Apply EMA smoothing for BOUNDING_BOX
  if (measurement.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return;
  }

  // Get shape dimensions
  const Eigen::Vector3d meas{
    measurement.shape.dimensions.x, measurement.shape.dimensions.y, measurement.shape.dimensions.z};

  // Initialize EMA if not already done
  if (!initialized_) {
    initialize(meas);
    return;
  }

  // Update shape dimensions using dual-rate EMA
  Eigen::Vector3d rel = (meas - value_).cwiseAbs().cwiseQuotient(value_.cwiseMax(1e-3));
  if (rel.maxCoeff() < shape_variation_threshold_) {
    value_ = alpha_strong_ * meas + (1.0 - alpha_strong_) * value_;
    ++stable_streak_;
    if (stable_streak_ >= stable_streak_threshold_) {
      stable_ = true;
    }
  } else {
    stable_streak_ = 0;
    stable_ = false;
    // Use weaker update even when measurement shape is noisy
    value_ = alpha_weak_ * meas + (1.0 - alpha_weak_) * value_;
  }
}

autoware_perception_msgs::msg::Shape ExponentialMovingAverageShape::getShape() const
{
  // For non-BOUNDING_BOX types, return the latest shape as-is (no smoothing)
  if (latest_shape_.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return latest_shape_;
  }

  // For BOUNDING_BOX type, return smoothed dimensions
  autoware_perception_msgs::msg::Shape shape;
  shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;

  // Set dimensions from smoothed EMA values
  shape.dimensions.x = value_(0);  // length
  shape.dimensions.y = value_(1);  // width
  shape.dimensions.z = value_(2);  // height

  return shape;
}

}  // namespace autoware::multi_object_tracker
