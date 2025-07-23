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

#include <deque>
#include <memory>
#include <string>
#include <utility>

namespace autoware::system::sensor_to_control_latency_checker
{

namespace
{

bool has_valid_data(const std::deque<TimestampedValue> & history)
{
  return !history.empty();
}

double get_latest_value(const std::deque<TimestampedValue> & history)
{
  if (!has_valid_data(history)) {
    return 0.0;
  }
  return history.back().value;
}

rclcpp::Time get_latest_timestamp(const std::deque<TimestampedValue> & history)
{
  if (history.empty()) {
    return rclcpp::Time(0);
  }
  return history.back().timestamp;
}

};  // namespace

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
        &SensorToControlLatencyCheckerNode::on_meas_to_tracked_object, this,
        std::placeholders::_1));

  processing_time_prediction_sub_ =
    create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/input/processing_time_prediction", 10,
      std::bind(
        &SensorToControlLatencyCheckerNode::on_processing_time_prediction, this,
        std::placeholders::_1));

  validation_status_sub_ =
    create_subscription<autoware_planning_validator::msg::PlanningValidatorStatus>(
      "~/input/validation_status", 10,
      std::bind(
        &SensorToControlLatencyCheckerNode::on_validation_status, this, std::placeholders::_1));

  control_component_latency_sub_ =
    create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/input/processing_time_control", 10,
      std::bind(
        &SensorToControlLatencyCheckerNode::on_control_component_latency, this,
        std::placeholders::_1));

  // Create publishers
  total_latency_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/output/total_latency_ms", 10);

  // Create debug publisher
  debug_publisher_ = std::make_unique<autoware::universe_utils::DebugPublisher>(
    this, "sensor_to_control_latency_checker");

  // Setup diagnostic updater
  diagnostic_updater_.setHardwareID("sensor_to_control_latency_checker");
  diagnostic_updater_.add(
    "Total Latency", this, &SensorToControlLatencyCheckerNode::check_total_latency);

  // Create timer
  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
    std::bind(&SensorToControlLatencyCheckerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "SensorToControlLatencyCheckerNode initialized");
}

void SensorToControlLatencyCheckerNode::on_meas_to_tracked_object(
  const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg)
{
  update_history(meas_to_tracked_object_history_, msg->stamp, msg->data);
  RCLCPP_DEBUG(get_logger(), "Received meas_to_tracked_object_ms: %.2f", msg->data);
}

void SensorToControlLatencyCheckerNode::on_processing_time_prediction(
  const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg)
{
  update_history(map_based_prediction_processing_time_history_, msg->stamp, msg->data);
  RCLCPP_DEBUG(get_logger(), "Received map_based_prediction processing_time_ms: %.2f", msg->data);
}

void SensorToControlLatencyCheckerNode::on_validation_status(
  const autoware_planning_validator::msg::PlanningValidatorStatus::ConstSharedPtr msg)
{
  // latency published by the planning validator is in seconds
  update_history(planning_component_latency_history_, msg->stamp, msg->latency * 1e3);
  RCLCPP_DEBUG(get_logger(), "Received planning_component_latency_ms: %.2f", msg->latency * 1e3);
}

void SensorToControlLatencyCheckerNode::on_control_component_latency(
  const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg)
{
  // latency published by the raw_vehicle_cmd_converter is in seconds
  update_history(control_component_latency_history_, msg->stamp, msg->data * 1e3);
  RCLCPP_DEBUG(get_logger(), "Received control_component_latency_ms: %.2f", msg->data * 1e3);
}

void SensorToControlLatencyCheckerNode::on_timer()
{
  calculate_total_latency();

  publish_total_latency();

  // Update diagnostics
  diagnostic_updater_.force_update();
}

void SensorToControlLatencyCheckerNode::calculate_total_latency()
{
  total_latency_ms_ = 0.0;

  // Get control_component_latency data (most recent)
  double control_component_latency_ms = 0.0;
  rclcpp::Time control_component_latency_timestamp = rclcpp::Time(0);
  if (has_valid_data(control_component_latency_history_)) {
    control_component_latency_ms = get_latest_value(control_component_latency_history_);
    control_component_latency_timestamp = get_latest_timestamp(control_component_latency_history_);
    total_latency_ms_ += control_component_latency_ms;
  }

  // Get planning_component_latency data
  // Condition: planning_timestamp + planning_latency < control_timestamp + control_latency
  double planning_component_latency_ms = 0.0;
  rclcpp::Time planning_component_latency_timestamp = rclcpp::Time(0);
  if (has_valid_data(planning_component_latency_history_)) {
    // Calculate target time: control_timestamp + control_latency
    rclcpp::Time control_end_time =
      control_component_latency_timestamp +
      rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(control_component_latency_ms * 1e6));

    // Find the most recent planning data where planning_timestamp + planning_latency <
    // control_end_time
    for (auto it = planning_component_latency_history_.rbegin();
         it != planning_component_latency_history_.rend(); ++it) {
      rclcpp::Time planning_end_time =
        it->timestamp + rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(it->value * 1e6));

      if (is_timestamp_older(planning_end_time, control_end_time)) {
        planning_component_latency_ms = it->value;
        planning_component_latency_timestamp = it->timestamp;
        total_latency_ms_ += planning_component_latency_ms;
        break;
      }
    }
  }

  // Get prediction processing_time data
  // Condition: prediction_timestamp + prediction_latency < planning_timestamp + planning_latency
  double map_based_prediction_processing_time_ms = 0.0;
  rclcpp::Time map_based_prediction_processing_time_timestamp = rclcpp::Time(0);
  if (
    has_valid_data(map_based_prediction_processing_time_history_) &&
    planning_component_latency_ms > 0.0) {
    // Calculate target time: planning_timestamp + planning_latency
    rclcpp::Time planning_end_time =
      planning_component_latency_timestamp +
      rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(planning_component_latency_ms * 1e6));

    // Find the most recent prediction data where prediction_timestamp + prediction_latency <
    // planning_end_time
    for (auto it = map_based_prediction_processing_time_history_.rbegin();
         it != map_based_prediction_processing_time_history_.rend(); ++it) {
      rclcpp::Time prediction_end_time =
        it->timestamp + rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(it->value * 1e6));

      if (is_timestamp_older(prediction_end_time, planning_end_time)) {
        map_based_prediction_processing_time_ms = it->value;
        map_based_prediction_processing_time_timestamp = it->timestamp;
        total_latency_ms_ += map_based_prediction_processing_time_ms;
        break;
      }
    }
  }

  // Get meas_to_tracked_object data
  // Condition: tracking_timestamp + tracking_latency < prediction_timestamp + prediction_latency
  double meas_to_tracked_object_ms = 0.0;
  if (
    has_valid_data(meas_to_tracked_object_history_) &&
    map_based_prediction_processing_time_ms > 0.0) {
    // Calculate target time: prediction_timestamp + prediction_latency
    rclcpp::Time prediction_end_time = map_based_prediction_processing_time_timestamp +
                                       rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(
                                         map_based_prediction_processing_time_ms * 1e6));

    // Find the most recent tracking data where tracking_timestamp + tracking_latency <
    // prediction_end_time
    for (auto it = meas_to_tracked_object_history_.rbegin();
         it != meas_to_tracked_object_history_.rend(); ++it) {
      rclcpp::Time tracking_end_time =
        it->timestamp + rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(it->value * 1e6));

      if (is_timestamp_older(tracking_end_time, prediction_end_time)) {
        meas_to_tracked_object_ms = it->value;
        total_latency_ms_ += meas_to_tracked_object_ms;
        break;
      }
    }
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Total latency calculation (cumulative time-ordered): control_component_latency=%.2f + "
    "planning_component_latency=%.2f + map_based_prediction_processing_time=%.2f + "
    "meas_to_tracked_object=%.2f = %.2f ms",
    control_component_latency_ms, planning_component_latency_ms,
    map_based_prediction_processing_time_ms, meas_to_tracked_object_ms, total_latency_ms_);

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

void SensorToControlLatencyCheckerNode::publish_total_latency()
{
  // Publish total latency
  auto total_latency_msg = std::make_unique<autoware_internal_debug_msgs::msg::Float64Stamped>();
  total_latency_msg->stamp = now();
  total_latency_msg->data = total_latency_ms_;
  total_latency_pub_->publish(std::move(total_latency_msg));

  // Publish debug information (using latest values and timestamps with initialization check)
  double meas_to_tracked_object_ms = get_latest_value(meas_to_tracked_object_history_);
  double map_based_prediction_processing_time_ms =
    get_latest_value(map_based_prediction_processing_time_history_);
  double planning_component_latency_ms = get_latest_value(planning_component_latency_history_);
  double control_component_latency_ms = get_latest_value(control_component_latency_history_);

  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/meas_to_tracked_object_ms", meas_to_tracked_object_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/map_based_prediction_processing_time_ms", map_based_prediction_processing_time_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/planning_component_latency_ms", planning_component_latency_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/control_component_latency_ms", control_component_latency_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/total_latency_ms", total_latency_ms_);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Total sensor-to-control latency: %.2f ms (threshold: %.2f ms)", total_latency_ms_,
    latency_threshold_ms_);
}

void SensorToControlLatencyCheckerNode::check_total_latency(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Get latest values
  double meas_to_tracked_object_ms = get_latest_value(meas_to_tracked_object_history_);
  double map_based_prediction_processing_time_ms =
    get_latest_value(map_based_prediction_processing_time_history_);
  double planning_component_latency_ms = get_latest_value(planning_component_latency_history_);
  double control_component_latency_ms = get_latest_value(control_component_latency_history_);

  stat.add("Total Latency (ms)", total_latency_ms_);
  stat.add("Threshold (ms)", latency_threshold_ms_);
  stat.add("meas_to_tracked_object_ms", meas_to_tracked_object_ms);
  stat.add("map_based_prediction_processing_time_ms", map_based_prediction_processing_time_ms);
  stat.add("planning_component_latency_ms", planning_component_latency_ms);
  stat.add("control_component_latency_ms", control_component_latency_ms);

  // Check if all data is initialized
  bool all_data_initialized = has_valid_data(meas_to_tracked_object_history_) &&
                              has_valid_data(map_based_prediction_processing_time_history_) &&
                              has_valid_data(planning_component_latency_history_) &&
                              has_valid_data(control_component_latency_history_);

  const auto append = [&](auto & base, const auto & str) {
    if (!base.empty()) {
      base += ", ";
    }
    base += str;
  };
  if (!all_data_initialized) {
    // Add detailed information about which data is not initialized
    std::string uninitialized_data;
    if (!has_valid_data(meas_to_tracked_object_history_)) {
      append(uninitialized_data, "meas_to_tracked_object");
    }
    if (!has_valid_data(map_based_prediction_processing_time_history_)) {
      append(uninitialized_data, "map_based_prediction_processing_time");
    }
    if (!has_valid_data(planning_component_latency_history_)) {
      append(uninitialized_data, "planning_component_latency");
    }
    if (!has_valid_data(control_component_latency_history_)) {
      append(uninitialized_data, "control_component_latency");
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

void SensorToControlLatencyCheckerNode::update_history(
  std::deque<TimestampedValue> & history, const rclcpp::Time & timestamp, double value) const
{
  // Add new value to history
  history.emplace_back(timestamp, value);

  // Remove old data if window size is exceeded
  while (history.size() > window_size_) {
    history.pop_front();
  }
}

bool SensorToControlLatencyCheckerNode::is_timestamp_older(
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
