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
#include <string>
#include <utility>
#include <vector>

namespace autoware::system::sensor_to_control_latency_checker
{

SensorToControlLatencyCheckerNode::SensorToControlLatencyCheckerNode(
  const rclcpp::NodeOptions & options)
: Node("sensor_to_control_latency_checker", options), diagnostic_updater_(this)
{
  update_rate_ = declare_parameter<double>("update_rate");
  latency_threshold_ms_ = declare_parameter<double>("latency_threshold_ms");
  window_size_ = declare_parameter<int>("window_size");

  // Initialize offset parameters
  sensor_offset_ms_ = declare_parameter<double>("sensor_offset_ms");
  perception_offset_ms_ = declare_parameter<double>("perception_offset_ms");
  planning_offset_ms_ = declare_parameter<double>("planning_offset_ms");
  control_offset_ms_ = declare_parameter<double>("control_offset_ms");
  vehicle_offset_ms_ = declare_parameter<double>("vehicle_offset_ms");

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

  control_system_latency_sub_ =
    create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/input/processing_time_control", 10,
      std::bind(
        &SensorToControlLatencyCheckerNode::onControlSystemLatency, this, std::placeholders::_1));

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
  updateHistory(map_based_prediction_processing_time_history_, msg->stamp, msg->data);
  RCLCPP_DEBUG(get_logger(), "Received map_based_prediction processing_time_ms: %.2f", msg->data);
}

void SensorToControlLatencyCheckerNode::onValidationStatus(
  const autoware_planning_validator::msg::PlanningValidatorStatus::ConstSharedPtr msg)
{
  // latency published by the planning validator is in seconds
  updateHistory(planning_system_latency_history_, msg->stamp, msg->latency * 1e3);
  RCLCPP_DEBUG(get_logger(), "Received planning_system_latency_ms: %.2f", msg->latency * 1e3);
}

void SensorToControlLatencyCheckerNode::onControlSystemLatency(
  const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg)
{
  // latency published by the raw_vehicle_cmd_converter is in seconds
  updateHistory(control_system_latency_history_, msg->stamp, msg->data * 1e3);
  RCLCPP_DEBUG(get_logger(), "Received control_system_latency_ms: %.2f", msg->data * 1e3);
}

void SensorToControlLatencyCheckerNode::onTimer()
{
  calculateTotalLatency();

  publishTotalLatency();

  // Update diagnostics
  diagnostic_updater_.force_update();
}

void SensorToControlLatencyCheckerNode::calculateTotalLatency()
{
  total_latency_ms_ = 0.0;

  // Get control_system_latency data (most recent)
  double control_system_latency_ms = 0.0;
  rclcpp::Time control_system_latency_timestamp = rclcpp::Time(0);
  if (hasValidData(control_system_latency_history_)) {
    control_system_latency_ms = getLatestValue(control_system_latency_history_);
    control_system_latency_timestamp = getLatestTimestamp(control_system_latency_history_);
    total_latency_ms_ += control_system_latency_ms;
  }

  // Get processing_time_latency data (older than control_system_latency_timestamp)
  double planning_system_latency_ms = 0.0;
  rclcpp::Time planning_system_latency_timestamp = rclcpp::Time(0);
  if (hasValidData(planning_system_latency_history_)) {
    // Find the most recent value that is older than control_system_latency_timestamp
    for (auto it = planning_system_latency_history_.rbegin();
         it != planning_system_latency_history_.rend(); ++it) {
      if (isTimestampOlder(it->timestamp, control_system_latency_timestamp)) {
        planning_system_latency_ms = it->value;
        planning_system_latency_timestamp = it->timestamp;
        total_latency_ms_ += planning_system_latency_ms;
        break;
      }
    }
  }

  // Get processing_time data (older than planning_system_latency_timestamp)
  double map_based_prediction_processing_time_ms = 0.0;
  rclcpp::Time map_based_prediction_processing_time_timestamp = rclcpp::Time(0);
  if (hasValidData(map_based_prediction_processing_time_history_)) {
    // Find the most recent value that is older than planning_system_latency_timestamp
    for (auto it = map_based_prediction_processing_time_history_.rbegin();
         it != map_based_prediction_processing_time_history_.rend(); ++it) {
      if (isTimestampOlder(it->timestamp, planning_system_latency_timestamp)) {
        map_based_prediction_processing_time_ms = it->value;
        map_based_prediction_processing_time_timestamp = it->timestamp;
        total_latency_ms_ += map_based_prediction_processing_time_ms;
        break;
      }
    }
  }

  // Get meas_to_tracked_object data (older than map_based_prediction_processing_time_timestamp)
  double meas_to_tracked_object_ms = 0.0;
  if (hasValidData(meas_to_tracked_object_history_)) {
    // Find the most recent value that is older than map_based_prediction_processing_time_timestamp
    for (auto it = meas_to_tracked_object_history_.rbegin();
         it != meas_to_tracked_object_history_.rend(); ++it) {
      if (isTimestampOlder(it->timestamp, map_based_prediction_processing_time_timestamp)) {
        meas_to_tracked_object_ms = it->value;
        total_latency_ms_ += meas_to_tracked_object_ms;
        break;
      }
    }
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Total latency calculation (timestamp-ordered): control_system_latency=%.2f + "
    "planning_system_latency=%.2f + map_based_prediction_processing_time=%.2f + "
    "meas_to_tracked_object=%.2f = %.2f ms",
    control_system_latency_ms, planning_system_latency_ms, map_based_prediction_processing_time_ms,
    meas_to_tracked_object_ms, total_latency_ms_);

  // Add offset processing times for each layer
  total_latency_ms_ += sensor_offset_ms_;
  total_latency_ms_ += perception_offset_ms_;
  total_latency_ms_ += planning_offset_ms_;
  total_latency_ms_ += control_offset_ms_;
  total_latency_ms_ += vehicle_offset_ms_;

  RCLCPP_DEBUG(
    get_logger(),
    "Total latency with offsets: %.2f ms (sensor_offset=%.2f + perception_offset=%.2f + "
    "planning_offset=%.2f + control_offset=%.2f + vehicle_offset=%.2f)",
    total_latency_ms_, sensor_offset_ms_, perception_offset_ms_, planning_offset_ms_,
    control_offset_ms_, vehicle_offset_ms_);
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
  double map_based_prediction_processing_time_ms =
    hasValidData(map_based_prediction_processing_time_history_)
      ? getLatestValue(map_based_prediction_processing_time_history_)
      : 0.0;
  double planning_system_latency_ms = hasValidData(planning_system_latency_history_)
                                        ? getLatestValue(planning_system_latency_history_)
                                        : 0.0;
  double control_system_latency_ms = hasValidData(control_system_latency_history_)
                                       ? getLatestValue(control_system_latency_history_)
                                       : 0.0;

  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/meas_to_tracked_object_ms", meas_to_tracked_object_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/map_based_prediction_processing_time_ms", map_based_prediction_processing_time_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/planning_system_latency_ms", planning_system_latency_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/control_system_latency_ms", control_system_latency_ms);
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
  // Get latest values
  double meas_to_tracked_object_ms = hasValidData(meas_to_tracked_object_history_)
                                       ? getLatestValue(meas_to_tracked_object_history_)
                                       : 0.0;
  double map_based_prediction_processing_time_ms =
    hasValidData(map_based_prediction_processing_time_history_)
      ? getLatestValue(map_based_prediction_processing_time_history_)
      : 0.0;
  double planning_system_latency_ms = hasValidData(planning_system_latency_history_)
                                        ? getLatestValue(planning_system_latency_history_)
                                        : 0.0;
  double control_system_latency_ms = hasValidData(control_system_latency_history_)
                                       ? getLatestValue(control_system_latency_history_)
                                       : 0.0;

  stat.add("Total Latency (ms)", total_latency_ms_);
  stat.add("Threshold (ms)", latency_threshold_ms_);
  stat.add("meas_to_tracked_object_ms", meas_to_tracked_object_ms);
  stat.add("map_based_prediction_processing_time_ms", map_based_prediction_processing_time_ms);
  stat.add("planning_system_latency_ms", planning_system_latency_ms);
  stat.add("control_system_latency_ms", control_system_latency_ms);

  // Check if all data is initialized
  bool all_data_initialized = hasValidData(meas_to_tracked_object_history_) &&
                              hasValidData(map_based_prediction_processing_time_history_) &&
                              hasValidData(planning_system_latency_history_) &&
                              hasValidData(control_system_latency_history_);

  if (!all_data_initialized) {
    // Add detailed information about which data is not initialized
    std::string uninitialized_data = "";
    if (!hasValidData(meas_to_tracked_object_history_)) {
      if (!uninitialized_data.empty()) uninitialized_data += ", ";
      uninitialized_data += "meas_to_tracked_object";
    }
    if (!hasValidData(map_based_prediction_processing_time_history_)) {
      if (!uninitialized_data.empty()) uninitialized_data += ", ";
      uninitialized_data += "map_based_prediction_processing_time";
    }
    if (!hasValidData(planning_system_latency_history_)) {
      if (!uninitialized_data.empty()) uninitialized_data += ", ";
      uninitialized_data += "planning_system_latency";
    }
    if (!hasValidData(control_system_latency_history_)) {
      if (!uninitialized_data.empty()) uninitialized_data += ", ";
      uninitialized_data += "control_system_latency";
    }

    stat.add("uninitialized_data", uninitialized_data);
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Some latency data not yet initialized: " + uninitialized_data);
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
  // Add new value to history
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

bool SensorToControlLatencyCheckerNode::isTimestampOlder(
  const rclcpp::Time & timestamp1, const rclcpp::Time & timestamp2) const
{
  try {
    return timestamp1 < timestamp2;
  } catch (const std::runtime_error & e) {
    // If timestamps have different time sources, compare nanoseconds directly
    RCLCPP_DEBUG(get_logger(), "Timestamp comparison failed, using nanoseconds: %s", e.what());
    return timestamp1.nanoseconds() < timestamp2.nanoseconds();
  }
}

}  // namespace autoware::system::sensor_to_control_latency_checker

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::system::sensor_to_control_latency_checker::SensorToControlLatencyCheckerNode)
