// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__PERCEPTION_ONLINE_EVALUATOR__PERCEPTION_ONLINE_EVALUATOR_NODE_HPP_
#define AUTOWARE__PERCEPTION_ONLINE_EVALUATOR__PERCEPTION_ONLINE_EVALUATOR_NODE_HPP_

#include "autoware/perception_online_evaluator/metrics_calculator.hpp"
#include "autoware/perception_online_evaluator/parameters.hpp"
#include "autoware/perception_online_evaluator/perception_analytics_calculator.hpp"
#include "autoware_utils/math/accumulator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_perception_msgs/msg/object_classification.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <array>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::perception_diagnostics
{
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_utils::Accumulator;
using nav_msgs::msg::Odometry;
using TFMessage = tf2_msgs::msg::TFMessage;

using MarkerArray = visualization_msgs::msg::MarkerArray;

/**
 * @brief Node for perception evaluation
 */
class PerceptionOnlineEvaluatorNode : public rclcpp::Node
{
public:
  explicit PerceptionOnlineEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~PerceptionOnlineEvaluatorNode() {}

  /**
   * @brief callback on receiving a dynamic objects array
   * @param [in] objects_msg received dynamic object array message
   */
  void onObjects(const PredictedObjects::ConstSharedPtr objects_msg);

  /**
   * @brief Convert metric statistic to `tier4_metric_msgs::msg::Metric` and append to
   * `tier4_metric_msgs::msg::MetricArray`.
   *
   * @param metric Metric name.
   * @param metric_stat Metric statistic.
   * @param metrics_msg Metrics value container.
   */
  void toMetricMsg(
    const std::string & metric, const Accumulator<double> & metric_stat,
    tier4_metric_msgs::msg::MetricArray & metrics_msg) const;

  /**
   * @brief Convert metric value to `tier4_metric_msgs::msg::Metric` and append to
   * `tier4_metric_msgs::msg::MetricArray
   *
   * @param metric Metric name.
   * @param metric_stat Metric value.
   * @param metrics_msg Metrics value container.
   */
  void toMetricMsg(
    const std::string & metric, const double metric_value,
    tier4_metric_msgs::msg::MetricArray & metrics_msg) const;

private:
  // Flags to manager node outputs
  bool enable_online_evaluation_{false};
  bool enable_perception_analytics_{false};

  // Label list
  std::vector<uint8_t> label_list_{
    ObjectClassification::UNKNOWN, ObjectClassification::CAR,
    ObjectClassification::TRUCK,   ObjectClassification::BUS,
    ObjectClassification::TRAILER, ObjectClassification::MOTORCYCLE,
    ObjectClassification::BICYCLE, ObjectClassification::PEDESTRIAN,
  };

  // Subscribers (for both online evaluation and MOB)
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;

  // Publishers (for online evaluation)
  rclcpp::Publisher<tier4_metric_msgs::msg::MetricArray>::SharedPtr metrics_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  // Subscribers and publishers (for MOB)
  rclcpp::Subscription<Float64Stamped>::SharedPtr meas_to_tracked_latency_sub_;
  rclcpp::Subscription<Float64Stamped>::SharedPtr prediction_latency_sub_;
  rclcpp::Publisher<tier4_metric_msgs::msg::MetricArray>::SharedPtr perception_analytics_pub_;

  // Latency cache (by topic id)
  std::array<double, autoware::perception_diagnostics::LATENCY_TOPIC_NUM> latencies_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

  // Parameters
  std::shared_ptr<Parameters> parameters_;
  void initParameter();
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // Metrics Calculator
  MetricsCalculator metrics_calculator_;
  void publishMetrics();
  PerceptionAnalyticsCalculator perception_analytics_calculator_;
  void publishPerceptionAnalytics();

  // Debug
  void publishDebugMarker();
};
}  // namespace autoware::perception_diagnostics

#endif  // AUTOWARE__PERCEPTION_ONLINE_EVALUATOR__PERCEPTION_ONLINE_EVALUATOR_NODE_HPP_
