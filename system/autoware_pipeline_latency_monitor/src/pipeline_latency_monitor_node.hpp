// Copyright 2025 The Autoware Contributors
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

#ifndef PIPELINE_LATENCY_MONITOR_NODE_HPP_
#define PIPELINE_LATENCY_MONITOR_NODE_HPP_

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace autoware::system::pipeline_latency_monitor
{

/// @brief a timestamp and measured latency associated with a processing step
struct ProcessData
{
  rclcpp::Time timestamp;  // either the start or end of the process
  double latency_ms;       // measured duration of the process

  ProcessData(const rclcpp::Time & ts, double val) : timestamp(ts), latency_ms(val) {}
};

/// @brief meaning of a timestamp (either start or end of a process)
enum class TimestampMeaning { start, end };

/// @brief input for a process
struct ProcessInput
{
  std::string name;
  std::string topic;
  std::string topic_type;
  TimestampMeaning timestamp_meaning;
  double latency_multiplier;  // used when the received value is not in millisecond
  std::deque<ProcessData> latency_history;
};

class PipelineLatencyMonitorNode : public rclcpp::Node
{
public:
  explicit PipelineLatencyMonitorNode(const rclcpp::NodeOptions & options);

private:
  // Parameters
  double update_rate_{};
  double latency_threshold_ms_{};
  size_t window_size_{};

  // Sequence of latency inputs
  std::vector<ProcessInput> input_sequence_;
  // Offsets to add to the total latency (ms)
  std::vector<double> latency_offsets_;
  // Current total latency
  double total_latency_ms_{};

  // Subscribers to the input topics
  std::vector<rclcpp::GenericSubscription::SharedPtr> generic_subscribers_;

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
  void on_timer();
  void calculate_total_latency();
  void publish_total_latency();
  void check_total_latency(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Helper functions
  void update_history(
    std::deque<ProcessData> & history, const rclcpp::Time & timestamp, double value);
  bool is_timestamp_older(const rclcpp::Time & timestamp1, const rclcpp::Time & timestamp2) const;
};

}  // namespace autoware::system::pipeline_latency_monitor

#endif  // PIPELINE_LATENCY_MONITOR_NODE_HPP_
