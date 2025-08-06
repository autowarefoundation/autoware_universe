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

#ifndef AUTOWARE__PERCEPTION_ONLINE_EVALUATOR__PERCEPTION_ANALYTICS_CALCULATOR_HPP_
#define AUTOWARE__PERCEPTION_ONLINE_EVALUATOR__PERCEPTION_ANALYTICS_CALCULATOR_HPP_

#include "tf2_ros/buffer.h"

#include "autoware_perception_msgs/msg/predicted_objects.hpp"

#include <array>
#include <memory>
#include <mutex>

namespace autoware::perception_diagnostics
{
using autoware_perception_msgs::msg::PredictedObjects;

// Number of labels in ObjectClassification
static constexpr size_t LABEL_NUM = 12;
// Number of latency topics and their ids
static constexpr size_t LATENCY_TOPIC_NUM = 2;
static constexpr size_t LATENCY_TOPIC_ID_MEAS_TO_TRACKED = 0;
static constexpr size_t LATENCY_TOPIC_ID_PREDICTION = 1;

/**
 * @brief Single-frame, fixed‐size perception metrics
 */
struct FrameMetrics
{
  uint32_t all_object_count = 0;
  std::array<uint32_t, LABEL_NUM> object_count_by_label{};
  std::array<double, LABEL_NUM> max_distance_by_label{};
  std::array<double, LATENCY_TOPIC_NUM> latency_by_topic_id{};
  double total_latency = 0.0;
};

/**
 * @brief Calculate single frame metrics for perception analytics
 *
 * Analytics contents:
 *  - total object count
 *  - object count by label
 *  - max distance by label (in base_link)
 *  - latency by topic (for nodes in pipeline)
 *  - total latency
 */
class PerceptionAnalyticsCalculator
{
public:
  PerceptionAnalyticsCalculator() = default;

  /**
   * @brief Store the latest objects for computation
   * @param objects     PredictedObjects for this frame
   */
  void setPredictedObjects(PredictedObjects::ConstSharedPtr objects);

  /**
   * @brief Store the latest node latencies for computation
   * @param latencies     Latencies for this frame
   */
  void setLatencies(const std::array<double, LATENCY_TOPIC_NUM> & latencies);

  /**
   * @brief Compute frame metrics
   * @param tf_buffer   TF buffer used for transforms
   * @return FrameMetrics
   */
  FrameMetrics calculate(const tf2_ros::Buffer & tf_buffer) const;

private:
  mutable std::mutex predicted_objects_mutex_;
  PredictedObjects::ConstSharedPtr predicted_objects_ptr_;
  std::array<double, LATENCY_TOPIC_NUM> latencies_{};
};

}  // namespace autoware::perception_diagnostics

#endif  // AUTOWARE__PERCEPTION_ONLINE_EVALUATOR__PERCEPTION_ANALYTICS_CALCULATOR_HPP_
