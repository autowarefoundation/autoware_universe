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

#ifndef AUTOWARE_SENSOR_TO_CONTROL_LATENCY_CHECKER__SENSOR_TO_CONTROL_LATENCY_CHECKER_NODE_HPP_
#define AUTOWARE_SENSOR_TO_CONTROL_LATENCY_CHECKER__SENSOR_TO_CONTROL_LATENCY_CHECKER_NODE_HPP_

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware_planning_validator/msg/planning_validator_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <deque>
#include <memory>
#include <string>

namespace autoware::system::sensor_to_control_latency_checker
{

struct TimestampedValue
{
  rclcpp::Time timestamp;
  double value;

  TimestampedValue(const rclcpp::Time & ts, double val) : timestamp(ts), value(val) {}
};

class SensorToControlLatencyCheckerNode : public rclcpp::Node
{
public:
  explicit SensorToControlLatencyCheckerNode(const rclcpp::NodeOptions & options);

private:
  // Parameters
  double update_rate_{};
  double latency_threshold_ms_{};
  int window_size_{};

  // Offset processing times for each layer (ms)
  double sensor_offset_ms_{};
  double perception_offset_ms_{};
  double planning_offset_ms_{};
  double control_offset_ms_{};
  double vehicle_offset_ms_{};

  // History of received values (with timestamps)
  std::deque<TimestampedValue> meas_to_tracked_object_history_{};
  std::deque<TimestampedValue> map_based_prediction_processing_time_history_{};
  std::deque<TimestampedValue> planning_system_latency_history_{};
  std::deque<TimestampedValue> control_system_latency_history_{};

  // Current total latency
  double total_latency_ms_{};

  // Subscribers
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    meas_to_tracked_object_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    processing_time_prediction_sub_;
  rclcpp::Subscription<autoware_planning_validator::msg::PlanningValidatorStatus>::SharedPtr
    validation_status_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    control_system_latency_sub_;

  // Publishers
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    total_latency_pub_;

  // Debug publisher
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_;

  // Diagnostic updater
  diagnostic_updater::Updater diagnostic_updater_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback functions
  void onMeasToTrackedObject(
    const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg);
  void onProcessingTimePrediction(
    const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg);
  void onValidationStatus(
    const autoware_planning_validator::msg::PlanningValidatorStatus::ConstSharedPtr msg);
  void onControlSystemLatency(
    const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg);
  void onTimer();
  void calculateTotalLatency();
  void publishTotalLatency();
  void checkTotalLatency(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Helper functions
  void updateHistory(
    std::deque<TimestampedValue> & history, const rclcpp::Time & timestamp, double value);
  double getLatestValue(const std::deque<TimestampedValue> & history) const;
  rclcpp::Time getLatestTimestamp(const std::deque<TimestampedValue> & history) const;
  bool hasValidData(const std::deque<TimestampedValue> & history) const;
  bool isTimestampOlder(const rclcpp::Time & timestamp1, const rclcpp::Time & timestamp2) const;
};

}  // namespace autoware::system::sensor_to_control_latency_checker

#endif  // AUTOWARE_SENSOR_TO_CONTROL_LATENCY_CHECKER__SENSOR_TO_CONTROL_LATENCY_CHECKER_NODE_HPP_
