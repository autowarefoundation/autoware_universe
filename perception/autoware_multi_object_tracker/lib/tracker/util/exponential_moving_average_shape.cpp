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

#include "autoware/multi_object_tracker/tracker/util/exponential_moving_average_shape.hpp"

namespace autoware::multi_object_tracker
{

ExponentialMovingAverageShape::ExponentialMovingAverageShape(
  double alpha, double shape_variation_threshold, size_t stable_streak_threshold,
  size_t unstable_streak_threshold)
: alpha_(alpha),
  shape_variation_threshold_(shape_variation_threshold),
  stable_streak_threshold_(stable_streak_threshold),
  unstable_streak_threshold_(unstable_streak_threshold)
{
}

void ExponentialMovingAverageShape::initialize(const Eigen::Vector3d & init, bool is_bbox)
{
  value_ = init;
  is_bbox_ = is_bbox;
  initialized_ = true;
  stable_ = false;
  resetStreaks();
}

void ExponentialMovingAverageShape::update(const Eigen::Vector3d & meas, bool is_bbox)
{
  // if not initialized or initialized by non-bbox and update by bbox
  if (!initialized_ || (is_bbox && !is_bbox_)) {
    initialize(meas, is_bbox);
  }
  // if not bbox update, skip update
  if (!is_bbox) {
    stable_ = false;
    return;
  }
  Eigen::Vector3d rel = (meas - value_).cwiseAbs().cwiseQuotient(value_.cwiseMax(1e-3));
  if (rel.maxCoeff() < shape_variation_threshold_) {
    value_ = alpha_ * meas + (1.0 - alpha_) * value_;
    ++stable_streak_;
    unstable_streak_ = 0;
    if (stable_streak_ >= stable_streak_threshold_) {
      stable_ = true;
    }
  } else {
    ++unstable_streak_;
    if (unstable_streak_ >= unstable_streak_threshold_) {
      initialize(meas, is_bbox);
    }
  }
}

bool ExponentialMovingAverageShape::isInitialized() const
{
  return initialized_;
}

bool ExponentialMovingAverageShape::isStable() const
{
  return stable_;
}

void ExponentialMovingAverageShape::clear()
{
  initialized_ = false;
}

bool ExponentialMovingAverageShape::getBBoxValue(Eigen::Vector3d & value) const
{
  value = value_;
  return is_bbox_;
}

void ExponentialMovingAverageShape::resetStreaks()
{
  stable_streak_ = 0;
  unstable_streak_ = 0;
}

}  // namespace autoware::multi_object_tracker
