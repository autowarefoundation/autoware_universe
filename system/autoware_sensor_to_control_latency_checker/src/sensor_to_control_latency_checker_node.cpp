// Copyright 2024 The Autoware Contributors
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

#include "autoware_sensor_to_control_latency_checker/sensor_to_control_latency_checker_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <deque>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::system::sensor_to_control_latency_checker
{

SensorToControlLatencyCheckerNode::SensorToControlLatencyCheckerNode(
  const rclcpp::NodeOptions & options)
: Node("sensor_to_control_latency_checker", options), diagnostic_updater_(this)
{
  // Get parameters (default values are set in header)
  update_rate_ = declare_parameter<double>("update_rate", update_rate_);
  latency_threshold_ms_ = declare_parameter<double>("latency_threshold_ms", latency_threshold_ms_);
  window_size_ = declare_parameter<int>("window_size", window_size_);

  // Create subscribers
  meas_to_tracked_object_sub_ =
    create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/input/processing_time_tracking", 10,
      std::bind(
        &SensorToControlLatencyCheckerNode::onMeasToTrackedObject, this, std::placeholders::_1));

  processing_time_prediction_sub_ =
    create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/input/processing_time_prediction", 10,
      std::bind(
        &SensorToControlLatencyCheckerNode::onProcessingTimePrediction, this,
        std::placeholders::_1));

  validation_status_sub_ =
    create_subscription<autoware_planning_validator::msg::PlanningValidatorStatus>(
      "~/input/validation_status", 10,
      std::bind(
        &SensorToControlLatencyCheckerNode::onValidationStatus, this, std::placeholders::_1));

  topic_delay_sub_ = create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/input/processing_time_control", 10,
    std::bind(&SensorToControlLatencyCheckerNode::onTopicDelay, this, std::placeholders::_1));

  // Create publishers
  total_latency_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/output/total_latency_ms", 10);

  // Create debug publisher
  debug_publisher_ = std::make_unique<autoware::universe_utils::DebugPublisher>(
    this, "sensor_to_control_latency_checker");

  // Setup diagnostic updater
  diagnostic_updater_.setHardwareID("sensor_to_control_latency_checker");
  diagnostic_updater_.add(
    "Total Latency", this, &SensorToControlLatencyCheckerNode::checkTotalLatency);

  // Create timer
  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
    std::bind(&SensorToControlLatencyCheckerNode::onTimer, this));

  RCLCPP_INFO(get_logger(), "SensorToControlLatencyCheckerNode initialized");
}

void SensorToControlLatencyCheckerNode::onMeasToTrackedObject(
  const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg)
{
  updateHistory(meas_to_tracked_object_history_, msg->stamp, msg->data);
  RCLCPP_DEBUG(get_logger(), "Received meas_to_tracked_object_ms: %.2f", msg->data);
}

void SensorToControlLatencyCheckerNode::onProcessingTimePrediction(
  const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg)
{
  updateHistory(processing_time_history_, msg->stamp, msg->data);
  RCLCPP_DEBUG(get_logger(), "Received processing_time_ms: %.2f", msg->data);
}

void SensorToControlLatencyCheckerNode::onValidationStatus(
  const autoware_planning_validator::msg::PlanningValidatorStatus::ConstSharedPtr msg)
{
  updateHistory(processing_time_latency_history_, msg->stamp, msg->latency);
  RCLCPP_DEBUG(get_logger(), "Received processing_time_latency_ms: %.2f", msg->latency);
}

void SensorToControlLatencyCheckerNode::onTopicDelay(
  const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg)
{
  updateHistory(topic_delay_history_, msg->stamp, msg->data);
  RCLCPP_DEBUG(get_logger(), "Received topic_delay_ms: %.2f", msg->data);
}

void SensorToControlLatencyCheckerNode::onTimer()
{
  // Calculate total latency
  calculateTotalLatency();

  // Publish results
  publishTotalLatency();

  // Update diagnostics
  diagnostic_updater_.force_update();
}

void SensorToControlLatencyCheckerNode::calculateTotalLatency()
{
  total_latency_ms_ = 0.0;

  // Get topic_delay data (most recent)
  double topic_delay_ms = 0.0;
  rclcpp::Time topic_delay_timestamp = rclcpp::Time(0);
  if (hasValidData(topic_delay_history_)) {
    topic_delay_ms = getLatestValue(topic_delay_history_);
    topic_delay_timestamp = getLatestTimestamp(topic_delay_history_);
    total_latency_ms_ += topic_delay_ms;
  }

  // Get processing_time_latency data (older than topic_delay_timestamp)
  double processing_time_latency_ms = 0.0;
  rclcpp::Time processing_time_latency_timestamp = rclcpp::Time(0);
  if (hasValidData(processing_time_latency_history_)) {
    // Find the most recent value that is older than topic_delay_timestamp
    for (auto it = processing_time_latency_history_.rbegin();
         it != processing_time_latency_history_.rend(); ++it) {
      if (it->timestamp < topic_delay_timestamp) {
        processing_time_latency_ms = it->value;
        processing_time_latency_timestamp = it->timestamp;
        total_latency_ms_ += processing_time_latency_ms;
        break;
      }
    }
  }

  // Get processing_time data (older than processing_time_latency_timestamp)
  double processing_time_ms = 0.0;
  rclcpp::Time processing_time_timestamp = rclcpp::Time(0);
  if (hasValidData(processing_time_history_)) {
    // Find the most recent value that is older than processing_time_latency_timestamp
    for (auto it = processing_time_history_.rbegin(); it != processing_time_history_.rend(); ++it) {
      if (it->timestamp < processing_time_latency_timestamp) {
        processing_time_ms = it->value;
        processing_time_timestamp = it->timestamp;
        total_latency_ms_ += processing_time_ms;
        break;
      }
    }
  }

  // Get meas_to_tracked_object data (older than processing_time_timestamp)
  double meas_to_tracked_object_ms = 0.0;
  if (hasValidData(meas_to_tracked_object_history_)) {
    // Find the most recent value that is older than processing_time_timestamp
    for (auto it = meas_to_tracked_object_history_.rbegin();
         it != meas_to_tracked_object_history_.rend(); ++it) {
      if (it->timestamp < processing_time_timestamp) {
        meas_to_tracked_object_ms = it->value;
        total_latency_ms_ += meas_to_tracked_object_ms;
        break;
      }
    }
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Total latency calculation (timestamp-ordered): topic_delay=%.2f + "
    "processing_time_latency=%.2f + processing_time=%.2f + meas_to_tracked_object=%.2f = %.2f ms",
    topic_delay_ms, processing_time_latency_ms, processing_time_ms, meas_to_tracked_object_ms,
    total_latency_ms_);
}

void SensorToControlLatencyCheckerNode::publishTotalLatency()
{
  // Publish total latency
  auto total_latency_msg = std::make_unique<autoware_internal_debug_msgs::msg::Float64Stamped>();
  total_latency_msg->stamp = now();
  total_latency_msg->data = total_latency_ms_;
  total_latency_pub_->publish(std::move(total_latency_msg));

  // Publish debug information (using latest values and timestamps with initialization check)
  double meas_to_tracked_object_ms = hasValidData(meas_to_tracked_object_history_)
                                       ? getLatestValue(meas_to_tracked_object_history_)
                                       : 0.0;
  double processing_time_ms =
    hasValidData(processing_time_history_) ? getLatestValue(processing_time_history_) : 0.0;
  double processing_time_latency_ms = hasValidData(processing_time_latency_history_)
                                        ? getLatestValue(processing_time_latency_history_)
                                        : 0.0;
  double topic_delay_ms =
    hasValidData(topic_delay_history_) ? getLatestValue(topic_delay_history_) : 0.0;

  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/meas_to_tracked_object_ms", meas_to_tracked_object_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", processing_time_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_latency_ms", processing_time_latency_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/topic_delay_ms", topic_delay_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/total_latency_ms", total_latency_ms_);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Total sensor-to-control latency: %.2f ms (threshold: %.2f ms)", total_latency_ms_,
    latency_threshold_ms_);
}

void SensorToControlLatencyCheckerNode::checkTotalLatency(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Get latest values (with initialization check)
  double meas_to_tracked_object_ms = hasValidData(meas_to_tracked_object_history_)
                                       ? getLatestValue(meas_to_tracked_object_history_)
                                       : 0.0;
  double processing_time_ms =
    hasValidData(processing_time_history_) ? getLatestValue(processing_time_history_) : 0.0;
  double processing_time_latency_ms = hasValidData(processing_time_latency_history_)
                                        ? getLatestValue(processing_time_latency_history_)
                                        : 0.0;
  double topic_delay_ms =
    hasValidData(topic_delay_history_) ? getLatestValue(topic_delay_history_) : 0.0;

  stat.add("Total Latency (ms)", total_latency_ms_);
  stat.add("Threshold (ms)", latency_threshold_ms_);
  stat.add("meas_to_tracked_object_ms", meas_to_tracked_object_ms);
  stat.add("processing_time_ms", processing_time_ms);
  stat.add("processing_time_latency_ms", processing_time_latency_ms);
  stat.add("topic_delay_ms", topic_delay_ms);

  // Also add history sizes and initialization status
  stat.add(
    "meas_to_tracked_object_history_size",
    static_cast<int>(getHistorySize(meas_to_tracked_object_history_)));
  stat.add(
    "processing_time_history_size", static_cast<int>(getHistorySize(processing_time_history_)));
  stat.add(
    "processing_time_latency_history_size",
    static_cast<int>(getHistorySize(processing_time_latency_history_)));
  stat.add("topic_delay_history_size", static_cast<int>(getHistorySize(topic_delay_history_)));

  stat.add("meas_to_tracked_object_initialized", hasValidData(meas_to_tracked_object_history_));
  stat.add("processing_time_initialized", hasValidData(processing_time_history_));
  stat.add("processing_time_latency_initialized", hasValidData(processing_time_latency_history_));
  stat.add("topic_delay_initialized", hasValidData(topic_delay_history_));

  // Check if all data is initialized
  bool all_data_initialized =
    hasValidData(meas_to_tracked_object_history_) && hasValidData(processing_time_history_) &&
    hasValidData(processing_time_latency_history_) && hasValidData(topic_delay_history_);

  if (!all_data_initialized) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Some latency data not yet initialized");
  } else if (total_latency_ms_ > latency_threshold_ms_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Total latency exceeds threshold");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK, "Total latency within acceptable range");
  }
}

void SensorToControlLatencyCheckerNode::updateHistory(
  std::deque<TimestampedValue> & history, const rclcpp::Time & timestamp, double value)
{
  // Add new value to history (direct construction)
  history.emplace_back(timestamp, value);

  // Remove old data if window size is exceeded
  while (static_cast<int>(history.size()) > window_size_) {
    history.pop_front();
  }
}

double SensorToControlLatencyCheckerNode::getLatestValue(
  const std::deque<TimestampedValue> & history) const
{
  if (history.empty()) {
    return 0.0;
  }
  return history.back().value;
}

rclcpp::Time SensorToControlLatencyCheckerNode::getLatestTimestamp(
  const std::deque<TimestampedValue> & history) const
{
  if (history.empty()) {
    return rclcpp::Time(0);
  }
  return history.back().timestamp;
}

bool SensorToControlLatencyCheckerNode::hasValidData(
  const std::deque<TimestampedValue> & history) const
{
  return !history.empty();
}

size_t SensorToControlLatencyCheckerNode::getHistorySize(
  const std::deque<TimestampedValue> & history) const
{
  return history.size();
}

}  // namespace autoware::system::sensor_to_control_latency_checker

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::system::sensor_to_control_latency_checker::SensorToControlLatencyCheckerNode)
