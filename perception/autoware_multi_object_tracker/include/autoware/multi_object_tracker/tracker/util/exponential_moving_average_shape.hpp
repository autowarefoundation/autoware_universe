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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UTIL__EXPONENTIAL_MOVING_AVERAGE_SHAPE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UTIL__EXPONENTIAL_MOVING_AVERAGE_SHAPE_HPP_

#include <Eigen/Core>

namespace autoware::multi_object_tracker
{

class ExponentialMovingAverageShape
{
public:
  ExponentialMovingAverageShape(
    double alpha, double shape_variation_threshold, size_t stable_streak_threshold,
    size_t unstable_streak_threshold);

  void initialize(const Eigen::Vector3d & init, bool is_bbox);
  void update(const Eigen::Vector3d & meas, bool is_bbox);

  // State
  bool isInitialized() const;
  bool isStable() const;

  void clear();
  bool getBBoxValue(Eigen::Vector3d & value) const;

private:
  void resetStreaks();

  double alpha_;
  double shape_variation_threshold_;
  size_t stable_streak_threshold_;
  size_t unstable_streak_threshold_;

  Eigen::Vector3d value_{0, 0, 0};
  bool initialized_{false};
  bool is_bbox_{false};
  bool stable_{false};
  size_t stable_streak_{0};
  size_t unstable_streak_{0};
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UTIL__EXPONENTIAL_MOVING_AVERAGE_SHAPE_HPP_