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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__UNSTABLE_SHAPE_FILTER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__UNSTABLE_SHAPE_FILTER_HPP_

#include "autoware/multi_object_tracker/object_model/types.hpp"

#include <Eigen/Core>

#include <autoware_perception_msgs/msg/shape.hpp>

namespace autoware::multi_object_tracker
{

class UnstableShapeFilter
{
private:
  bool initialized_;
  bool stable_;
  Eigen::Vector3d value_;
  double alpha_weak_;
  double alpha_strong_;
  double shape_variation_threshold_;
  int stable_streak_;
  int stable_streak_threshold_;
  autoware_perception_msgs::msg::Shape latest_shape_;

  // Consecutive frame tracking for EMA validity
  int consecutive_noisy_frames_;     // Count of consecutive noisy measurements (builds confidence)
  int consecutive_noisy_threshold_;  // Min consecutive noisy frames for reliable EMA
  int normal_frame_interruptions_;   // Count of normal frames interrupting noisy sequence

public:
  UnstableShapeFilter(
    double alpha_weak, double alpha_strong, double shape_variation_threshold,
    int stable_streak_threshold, int consecutive_noisy_threshold = 3);

  void initialize(const Eigen::Vector3d & initial_shape);
  void clear();
  bool isStable() const { return stable_; }
  autoware_perception_msgs::msg::Shape getShape() const;

  void processNoisyMeasurement(const types::DynamicObject & measurement);
  void processNormalMeasurement(const types::DynamicObject & measurement);
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__UNSTABLE_SHAPE_FILTER_HPP_
