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

#include "autoware/perception_online_evaluator/perception_analytics_publisher_node.hpp"

#include "autoware/object_recognition_utils/object_classification.hpp"
#include "autoware_utils/ros/parameter.hpp"
#include "autoware_utils/ros/update_param.hpp"

#include <glog/logging.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::perception_diagnostics
{
using autoware::object_recognition_utils::convertLabelToString;

PerceptionAnalyticsPublisherNode::PerceptionAnalyticsPublisherNode(
  const rclcpp::NodeOptions & node_options)
: Node("perception_analytics_publisher", node_options),
  parameters_(std::make_shared<AnalyticsParameters>())
{
  using std::placeholders::_1;

  if (!google::IsGoogleLoggingInitialized()) {
    google::InitGoogleLogging("perception_analytics_publisher_node");
    google::InstallFailureSignalHandler();
  }

  // Parameters
  initParameter();

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PerceptionAnalyticsPublisherNode::onParameter, this, std::placeholders::_1));

  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", 1, std::bind(&PerceptionAnalyticsPublisherNode::onObjects, this, _1));

  perception_analytics_pub_ =
    create_publisher<tier4_metric_msgs::msg::MetricArray>("~/perception_analytics", 1);

  const auto latency_topic_meas_to_tracked =
    this->get_parameter("meas_to_tracked_latency_topic_name").as_string();
  const auto latency_topic_prediction =
    this->get_parameter("prediction_latency_topic_name").as_string();
  meas_to_tracked_latency_sub_ = create_subscription<Float64Stamped>(
    latency_topic_meas_to_tracked, 1, [this](const Float64Stamped::ConstSharedPtr msg) {
      latencies_[LATENCY_TOPIC_ID_MEAS_TO_TRACKED] = msg->data;
    });
  prediction_latency_sub_ = create_subscription<Float64Stamped>(
    latency_topic_prediction, 1, [this](const Float64Stamped::ConstSharedPtr msg) {
      latencies_[LATENCY_TOPIC_ID_PREDICTION] = msg->data;
    });

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void PerceptionAnalyticsPublisherNode::publishPerceptionAnalytics()
{
  auto metrics = perception_analytics_calculator_.calculate(*tf_buffer_);

  // DiagnosticArray metrics_msg;
  tier4_metric_msgs::msg::MetricArray metrics_msg;

  // all object count
  metrics_msg.metric_array.emplace_back(
    tier4_metric_msgs::build<tier4_metric_msgs::msg::Metric>()
      .name("all_object_count")
      .unit("count")
      .value(std::to_string(metrics.all_object_count)));

  // object count per label
  for (auto & label : label_list_) {
    metrics_msg.metric_array.emplace_back(
      tier4_metric_msgs::build<tier4_metric_msgs::msg::Metric>()
        .name("object_count_" + convertLabelToString(label))
        .unit("count")
        .value(std::to_string(metrics.object_count_by_label[label])));
  }

  // max distance per label (value 0 for no object)
  for (auto & label : label_list_) {
    metrics_msg.metric_array.emplace_back(
      tier4_metric_msgs::build<tier4_metric_msgs::msg::Metric>()
        .name("max_distance_" + convertLabelToString(label))
        .unit("m")
        .value(std::to_string(metrics.max_distance_by_label[label])));
  }

  // latency by topic
  metrics_msg.metric_array.emplace_back(
    tier4_metric_msgs::build<tier4_metric_msgs::msg::Metric>()
      .name("meas_to_tracked_latency")
      .unit("ms")
      .value(std::to_string(metrics.latency_by_topic_id[LATENCY_TOPIC_ID_MEAS_TO_TRACKED])));
  metrics_msg.metric_array.emplace_back(
    tier4_metric_msgs::build<tier4_metric_msgs::msg::Metric>()
      .name("prediction_latency")
      .unit("ms")
      .value(std::to_string(metrics.latency_by_topic_id[LATENCY_TOPIC_ID_PREDICTION])));

  // total latency
  metrics_msg.metric_array.emplace_back(
    tier4_metric_msgs::build<tier4_metric_msgs::msg::Metric>()
      .name("total_latency")
      .unit("ms")
      .value(std::to_string(metrics.total_latency)));

  if (!metrics_msg.metric_array.empty()) {
    metrics_msg.stamp = now();
    perception_analytics_pub_->publish(metrics_msg);
  }
}

void PerceptionAnalyticsPublisherNode::onObjects(const PredictedObjects::ConstSharedPtr objects_msg)
{
  perception_analytics_calculator_.setPredictedObjects(objects_msg);
  perception_analytics_calculator_.setLatencies(latencies_);
  publishPerceptionAnalytics();
}

rcl_interfaces::msg::SetParametersResult PerceptionAnalyticsPublisherNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  auto & p = parameters_;

  update_param<std::string>(
    parameters, "meas_to_tracked_latency_topic_name", p->meas_to_tracked_latency_topic_name);
  update_param<std::string>(
    parameters, "prediction_latency_topic_name", p->prediction_latency_topic_name);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

void PerceptionAnalyticsPublisherNode::initParameter()
{
  using autoware_utils::get_or_declare_parameter;
  auto & p = parameters_;

  p->meas_to_tracked_latency_topic_name =
    get_or_declare_parameter<std::string>(*this, "meas_to_tracked_latency_topic_name");
  p->prediction_latency_topic_name =
    get_or_declare_parameter<std::string>(*this, "prediction_latency_topic_name");
}
}  // namespace autoware::perception_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::perception_diagnostics::PerceptionAnalyticsPublisherNode)
