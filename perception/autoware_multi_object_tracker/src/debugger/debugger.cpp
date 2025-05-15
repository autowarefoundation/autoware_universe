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

#include "debugger.hpp"

#include <algorithm>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::multi_object_tracker
{
TrackerDebugger::TrackerDebugger(
  rclcpp::Node & node, const std::string & frame_id,
  const std::vector<types::InputChannel> & channels_config)
: node_(node), diagnostic_updater_(&node), object_debugger_(frame_id, channels_config)
{
  // declare debug parameters to decide whether to publish debug topics
  loadParameters();
  // initialize debug publishers
  if (debug_settings_.publish_processing_time) {
    processing_time_publisher_ =
      std::make_unique<autoware_utils::DebugPublisher>(&node_, "multi_object_tracker");
  }

  if (debug_settings_.publish_tentative_objects) {
    debug_tentative_objects_pub_ =
      node_.create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
        "~/debug/tentative_objects", rclcpp::QoS{1});
  }

  if (debug_settings_.publish_debug_markers) {
    debug_objects_markers_pub_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/objects_markers", rclcpp::QoS{1});
  }

  // initialize timestamps
  const rclcpp::Time now = node_.now();
  last_input_stamp_ = now;
  stamp_process_start_ = now;
  stamp_process_end_ = now;
  stamp_publish_start_ = now;
  stamp_publish_output_ = now;
  last_non_warning_timestamp_ = now;
  // setup diagnostics
  setupDiagnostics();
}

void TrackerDebugger::loadParameters()
{
  try {
    debug_settings_.publish_processing_time =
      node_.declare_parameter<bool>("publish_processing_time");
    debug_settings_.publish_tentative_objects =
      node_.declare_parameter<bool>("publish_tentative_objects");
    debug_settings_.publish_debug_markers = node_.declare_parameter<bool>("publish_debug_markers");
    debug_settings_.diagnostics_warn_delay =
      node_.declare_parameter<double>("diagnostics_warn_delay");
    debug_settings_.diagnostics_error_delay =
      node_.declare_parameter<double>("diagnostics_error_delay");
    debug_settings_.diagnostics_warn_extrapolation =
      node_.declare_parameter<double>("diagnostics_warn_extrapolation");
    debug_settings_.diagnostics_error_extrapolation =
      node_.declare_parameter<double>("diagnostics_error_extrapolation");
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_.get_logger(), "Failed to declare parameter: %s", e.what());
    debug_settings_.publish_processing_time = false;
    debug_settings_.publish_tentative_objects = false;
    debug_settings_.publish_debug_markers = false;
    debug_settings_.diagnostics_warn_delay = 0.5;
    debug_settings_.diagnostics_error_delay = 1.0;
  }
}

void TrackerDebugger::setupDiagnostics()
{
  diagnostic_updater_.setHardwareID(node_.get_name());
  diagnostic_updater_.add("Tracker Timing Diagnostics", this, &TrackerDebugger::checkAllTiming);
  diagnostic_updater_.setPeriod(0.1);
}

void TrackerDebugger::updateDiagnosticValues(double min_extrapolation_time, size_t published_count)
{
  diagnostic_values_.min_extrapolation_time = min_extrapolation_time;
  diagnostic_values_.published_trackers_count = published_count;
  // Force update diagnostic values
  diagnostic_updater_.force_update();
}

void TrackerDebugger::publishTentativeObjects(
  const autoware_perception_msgs::msg::TrackedObjects & tentative_objects) const
{
  if (debug_settings_.publish_tentative_objects) {
    debug_tentative_objects_pub_->publish(tentative_objects);
  }
}

TrackerDebugger::TimingCheckResult TrackerDebugger::checkDelayTiming(double delay) const
{
  if (delay == 0.0) {
    return {"[OK] Not calculated", diagnostic_msgs::msg::DiagnosticStatus::OK};
  }
  if (delay < debug_settings_.diagnostics_warn_delay) {
    return {"[OK] Within limits", diagnostic_msgs::msg::DiagnosticStatus::OK};
  }
  if (delay < debug_settings_.diagnostics_error_delay) {
    return {"[WARN] Exceeded warn threshold", diagnostic_msgs::msg::DiagnosticStatus::WARN};
  }
  return {"[ERROR] Exceeded error threshold", diagnostic_msgs::msg::DiagnosticStatus::ERROR};
}

TrackerDebugger::TimingCheckResult TrackerDebugger::checkExtrapolationTiming(
  double extrapolation_time, const rclcpp::Time & timestamp)
{
  if (extrapolation_time <= debug_settings_.diagnostics_warn_extrapolation) {
    last_non_warning_timestamp_ = timestamp;
    return {
      "[OK] Extrapolation time is within safe limits. ",
      diagnostic_msgs::msg::DiagnosticStatus::OK};
  }

  // If this is the first time a warning occurs, initialize the timestamp
  if (last_non_warning_timestamp_.nanoseconds() == 0) {
    last_non_warning_timestamp_ = timestamp;
  }

  // Calculate consecutive warning duration in seconds
  const double consecutive_warning_duration_s =
    std::chrono::duration<double>(
      std::chrono::nanoseconds((timestamp - last_non_warning_timestamp_).nanoseconds()))
      .count();

  // Check if warnings have persisted beyond the allowed duration
  if (consecutive_warning_duration_s > debug_settings_.diagnostics_error_extrapolation) {
    return {
      "[ERROR] Extrapolation time exceeded the warning threshold of " +
        std::to_string(debug_settings_.diagnostics_warn_extrapolation) + " for " +
        std::to_string(consecutive_warning_duration_s) + " seconds (Threshold " +
        std::to_string(debug_settings_.diagnostics_error_extrapolation) + ")",
      diagnostic_msgs::msg::DiagnosticStatus::ERROR};
  }

  return {
    "[WARN] Extrapolation time exceeds warning threshold " +
      std::to_string(debug_settings_.diagnostics_warn_extrapolation),
    diagnostic_msgs::msg::DiagnosticStatus::WARN};
}

TrackerDebugger::TimingCheckResult TrackerDebugger::determineOverallTimingStatus(
  bool no_published_trackers, const TimingCheckResult & delay_result,
  const TimingCheckResult & extrapolation_result)
{
  if (no_published_trackers) {
    return {
      "[OK] No objects currently being tracked (normal operation when no detections)",
      diagnostic_msgs::msg::DiagnosticStatus::OK};
  }

  const uint8_t max_level = std::max(delay_result.level, extrapolation_result.level);

  if (max_level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    return {"[OK] All timing parameters are within safe limits.", max_level};
  }

  // Determine base message based on max severity level
  static const std::unordered_map<uint8_t, std::string> level_messages = {
    {diagnostic_msgs::msg::DiagnosticStatus::WARN, "[WARN] Timing warning: "},
    {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "[ERROR] Timing issue detected: "}};

  std::string message = level_messages.at(max_level);

  // Append specific issues
  if (delay_result.level == max_level) {
    message += "Detection delay exceeded threshold. ";
  }

  if (extrapolation_result.level == max_level) {
    message += (extrapolation_result.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR)
                 ? "Extrapolation warning persisted for too long! "
                 : "Extrapolation time exceeded warning threshold. ";
  }

  return {message, max_level};
}

// Time measurement functions
void TrackerDebugger::checkAllTiming(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Check initialization status
  if (!is_initialized_) {
    stat.add("Detection status", "Not initialized");
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Measurement time is not set.");
    return;
  }

  const double delay = pipeline_latency_ms_ / 1e3;  // [s]
  // Check if we have any published trackers
  const bool no_published_trackers = (diagnostic_values_.published_trackers_count == 0);

  // Check individual timing components
  const auto delay_result = checkDelayTiming(delay);
  const auto extrapolation_result =
    checkExtrapolationTiming(diagnostic_values_.min_extrapolation_time, node_.now());
  // Determine overall status
  const auto overall_result =
    determineOverallTimingStatus(no_published_trackers, delay_result, extrapolation_result);

  stat.add("Detection delay (s)", delay);
  stat.add("Detection status", delay_result.message);
  stat.add("Extrapolation time (s)", diagnostic_values_.min_extrapolation_time);
  stat.add("Extrapolation status", extrapolation_result.message);
  // Set the overall status based on the worst condition
  stat.summary(overall_result.level, overall_result.message);
}

void TrackerDebugger::startMeasurementTime(
  const rclcpp::Time & now, const rclcpp::Time & measurement_header_stamp)
{
  last_input_stamp_ = measurement_header_stamp;
  stamp_process_start_ = now;
  if (debug_settings_.publish_processing_time) {
    double input_latency_ms = (now - last_input_stamp_).seconds() * 1e3;
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/input_latency_ms", input_latency_ms);
  }
  // initialize debug time stamps
  if (!is_initialized_) {
    stamp_publish_output_ = now;
    is_initialized_ = true;
  }
}

void TrackerDebugger::endMeasurementTime(const rclcpp::Time & now)
{
  stamp_process_end_ = now;
}

void TrackerDebugger::startPublishTime(const rclcpp::Time & now)
{
  stamp_publish_start_ = now;
}

void TrackerDebugger::endPublishTime(const rclcpp::Time & now, const rclcpp::Time & object_time)
{
  // if the measurement time is not set, do not publish debug information
  if (!is_initialized_) return;

  // pipeline latency: time from sensor measurement to publish, used for 'checkDelay'
  pipeline_latency_ms_ = (now - last_input_stamp_).seconds() * 1e3;

  if (debug_settings_.publish_processing_time) {
    // processing latency: time between input and publish
    double processing_latency_ms = ((now - stamp_process_start_).seconds()) * 1e3;
    // processing time: only the time spent in the tracking processes
    double processing_time_ms = ((now - stamp_publish_start_).seconds() +
                                 (stamp_process_end_ - stamp_process_start_).seconds()) *
                                1e3;
    // cycle time: time between two consecutive publish
    double cyclic_time_ms = (now - stamp_publish_output_).seconds() * 1e3;
    // measurement to tracked-object time: time from the sensor measurement to the publishing object
    // time If there is not latency compensation, this value is zero
    double measurement_to_object_ms = (object_time - last_input_stamp_).seconds() * 1e3;

    // starting from the measurement time
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms_);
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_latency_ms", processing_latency_ms);
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/meas_to_tracked_object_ms", measurement_to_object_ms);
  }
  stamp_publish_output_ = now;
}

void TrackerDebugger::collectObjectInfo(
  const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
  const types::DynamicObjectList & detected_objects,
  const std::unordered_map<int, int> & direct_assignment,
  const std::unordered_map<int, int> & reverse_assignment)
{
  if (!debug_settings_.publish_debug_markers) return;
  object_debugger_.collect(
    message_time, list_tracker, detected_objects, direct_assignment, reverse_assignment);
}

// ObjectDebugger
void TrackerDebugger::publishObjectsMarkers()
{
  if (!debug_settings_.publish_debug_markers) return;

  visualization_msgs::msg::MarkerArray marker_message;
  marker_message.markers.clear();
  // process data
  object_debugger_.process();

  // publish markers
  object_debugger_.getMessage(marker_message);
  debug_objects_markers_pub_->publish(marker_message);

  // reset object data
  object_debugger_.reset();
}

}  // namespace autoware::multi_object_tracker
