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

#include "autoware/perception_online_evaluator/mob_metrics_calculator.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <numeric>

namespace autoware::perception_diagnostics
{

void MobMetricsCalculator::setPredictedObjects(const PredictedObjects & objects)
{
  predicted_objects_ = objects;
}

void MobMetricsCalculator::setLatencies(const std::array<double, LATENCY_TOPIC_NUM> & latencies)
{
  latencies_ = latencies;
}

FrameMetrics MobMetricsCalculator::computeMetrics(const tf2_ros::Buffer & tf_buffer) const
{
  FrameMetrics metrics;
  metrics.all_object_count = static_cast<uint32_t>(predicted_objects_.objects.size());

  // Compute object count by label
  for (const auto & object : predicted_objects_.objects) {
    const auto label =
      autoware::object_recognition_utils::getHighestProbLabel(object.classification);
    metrics.object_count_by_label[label]++;
  }

  // Store latency_by_topic_id and compute total_latency
  metrics.latency_by_topic_id = latencies_;
  metrics.total_latency = std::accumulate(latencies_.begin(), latencies_.end(), 0.0);

  // Skip max distance calculation if base_link transform is unavailable
  const auto objects_frame_id = predicted_objects_.header.frame_id;
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer.lookupTransform(
      "base_link", objects_frame_id, tf2::TimePointZero, tf2::Duration::zero());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      rclcpp::get_logger("MobMetricsCalculator"),
      "TF lookup failed, skipping max distance calculation.");
    return metrics;
  }

  // Compute max distance by label
  for (const auto & object : predicted_objects_.objects) {
    const auto label =
      autoware::object_recognition_utils::getHighestProbLabel(object.classification);

    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.header.frame_id = objects_frame_id;
    pose_in.pose = object.kinematics.initial_pose_with_covariance.pose;

    // Transform the object's pose into the 'base_link' coordinate frame
    tf2::doTransform(pose_in, pose_out, transform_stamped);

    const double dist = std::hypot(pose_out.pose.position.x, pose_out.pose.position.y);

    auto & max_dist = metrics.max_distance_by_label[label];
    if (dist > max_dist) {
      max_dist = dist;
    }
  }

  return metrics;
}

}  // namespace autoware::perception_diagnostics
