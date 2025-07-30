// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__PERCEPTION_ONLINE_EVALUATOR__MOB_METRICS_CALCULATOR_HPP_
#define AUTOWARE__PERCEPTION_ONLINE_EVALUATOR__MOB_METRICS_CALCULATOR_HPP_

#include "tf2_ros/buffer.h"

#include "autoware_perception_msgs/msg/predicted_objects.hpp"

#include <unordered_map>

namespace autoware::perception_diagnostics
{
using autoware_perception_msgs::msg::PredictedObjects;

struct FrameMetrics
{
  uint32_t total_count = 0;
  std::unordered_map<uint8_t, uint32_t> counts;
  std::unordered_map<uint8_t, double> max_distances;
};

/**
 * @brief One‐shot frame metrics calculator for MOB
 *
 * Calculates metrics:
 *  - total object count
 *  - per‐label object count
 *  - per‐label max distance (in base_link)
 */
class MobMetricsCalculator
{
public:
  MobMetricsCalculator() = default;

  /**
   * @brief Store the latest objects for computation
   * @param objects     PredictedObjects for this frame
   */
  void setPredictedObjects(const PredictedObjects & objects);

  /**
   * @brief Compute frame metrics
   * @param tf_buffer   TF buffer used for transforms
   * @return FrameMetrics
   */
  FrameMetrics computeMetrics(const tf2_ros::Buffer & tf_buffer) const;

private:
  PredictedObjects predicted_objects_;
};

}  // namespace autoware::perception_diagnostics

#endif  // AUTOWARE__PERCEPTION_ONLINE_EVALUATOR__MOB_METRICS_CALCULATOR_HPP_
