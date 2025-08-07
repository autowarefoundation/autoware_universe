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

#include "pipeline_latency_monitor_node.hpp"

#include <autoware_planning_validator/msg/planning_validator_status.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <deque>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::system::pipeline_latency_monitor
{

namespace
{

bool has_valid_data(const std::deque<ProcessData> & history)
{
  return !history.empty();
}

double get_latest_value(const std::deque<ProcessData> & history)
{
  if (!has_valid_data(history)) {
    return 0.0;
  }
  const double value = history.back().latency_ms;
  // Return 0.0 for negative values
  return value < 0.0 ? 0.0 : value;
}

rclcpp::Time get_latest_timestamp(const std::deque<ProcessData> & history)
{
  if (history.empty()) {
    return rclcpp::Time(0);
  }
  return history.back().timestamp;
}

};  // namespace

PipelineLatencyMonitorNode::PipelineLatencyMonitorNode(const rclcpp::NodeOptions & options)
: Node("pipeline_latency_monitor", options), diagnostic_updater_(this)
{
  update_rate_ = declare_parameter<double>("update_rate");
  latency_threshold_ms_ = declare_parameter<double>("latency_threshold_ms");
  window_size_ = declare_parameter<int>("window_size");

  const auto processing_steps =
    declare_parameter<std::vector<std::string>>("processing_steps.sequence");
  for (const auto & step : processing_steps) {
    const auto topic = declare_parameter<std::string>("processing_steps." + step + ".topic");
    const auto topic_type =
      declare_parameter<std::string>("processing_steps." + step + ".topic_type");
    const auto timestamp_meaning =
      declare_parameter<std::string>("processing_steps." + step + ".timestamp_meaning");
    const auto latency_multiplier =
      declare_parameter<double>("processing_steps." + step + ".latency_multiplier");
    const auto index = input_sequence_.size();
    ProcessInput & input = input_sequence_.emplace_back();
    input.name = step;
    input.topic = topic;
    input.topic_type = topic_type;
    input.timestamp_meaning =
      timestamp_meaning == "start" ? TimestampMeaning::start : TimestampMeaning::end;
    input.latency_multiplier = latency_multiplier;

    // generic callback
    const auto callback = [this, index](
                            const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg) {
      auto & input = this->input_sequence_[index];
      if (input.topic_type == "autoware_internal_debug_msgs/msg/Float64Stamped") {
        static const rclcpp::Serialization<autoware_internal_debug_msgs::msg::Float64Stamped>
          serialization;
        autoware_internal_debug_msgs::msg::Float64Stamped msg;
        serialization.deserialize_message(serialized_msg.get(), &msg);
        this->update_history(input.latency_history, msg.stamp, msg.data * input.latency_multiplier);
        RCLCPP_DEBUG(
          get_logger(), "Received %s: %.2f", input.name.c_str(),
          msg.data * input.latency_multiplier);
      } else if (input.topic_type == "autoware_planning_validator/msg/PlanningValidatorStatus") {
        static const rclcpp::Serialization<
          autoware_planning_validator::msg::PlanningValidatorStatus>
          serialization;
        autoware_planning_validator::msg::PlanningValidatorStatus msg;
        serialization.deserialize_message(serialized_msg.get(), &msg);
        this->update_history(
          input.latency_history, msg.stamp, msg.latency * input.latency_multiplier);
        RCLCPP_DEBUG(
          get_logger(), "Received %s: %.2f", input.name.c_str(),
          msg.latency * input.latency_multiplier);
      } else {
        throw std::runtime_error("unsupported message type " + input.topic_type);
      }
    };
    generic_subscribers_.push_back(
      create_generic_subscription(topic, topic_type, rclcpp::QoS(1), callback));
  }
  latency_offsets_ = declare_parameter<std::vector<double>>("latency_offsets_ms");

  // Create publishers
  total_latency_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/output/total_latency_ms", 10);

  // Create debug publisher
  debug_publisher_ =
    std::make_unique<autoware::universe_utils::DebugPublisher>(this, "pipeline_latency_monitor");

  // Setup diagnostic updater
  diagnostic_updater_.setHardwareID("pipeline_latency_monitor");
  diagnostic_updater_.add("Total Latency", this, &PipelineLatencyMonitorNode::check_total_latency);

  // Create timer
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
    std::bind(&PipelineLatencyMonitorNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "PipelineLatencyMonitorNode initialized");
}

void PipelineLatencyMonitorNode::on_timer()
{
  calculate_total_latency();

  publish_total_latency();

  // Update diagnostics
  diagnostic_updater_.force_update();
}

void PipelineLatencyMonitorNode::calculate_total_latency()
{
  if (input_sequence_.empty()) {
    return;
  }
  const auto to_duration = [](const double latency_ms) {
    return rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(latency_ms * 1e6));
  };
  std::stringstream debug_ss;
  total_latency_ms_ = 0.0;
  rclcpp::Time start_of_next_step;
  // we go through the steps in reverse
  auto step_it = input_sequence_.rbegin();
  // use the latest latency of the last step in the sequence without any constraint
  const auto & input = *step_it;
  if (has_valid_data(input.latency_history)) {
    const auto latency = get_latest_value(input.latency_history);
    total_latency_ms_ += latency;
    debug_ss << step_it->name << "=" << latency;
    start_of_next_step = get_latest_timestamp(input.latency_history);
    if (input.timestamp_meaning == TimestampMeaning::end) {
      start_of_next_step -= to_duration(latency);
    }
  } else {
    // skip step if no data available
    debug_ss << step_it->name << "=skipped";
    start_of_next_step = now();
  }
  // we go through the rest of the sequence in reverse order with the following constraint:
  // end of current step < start of next step
  for (step_it++; step_it != input_sequence_.rend(); ++step_it) {
    const auto & step_input = *step_it;
    bool found_valid_data = false;
    if (has_valid_data(step_input.latency_history)) {
      for (auto it = step_input.latency_history.rbegin(); it != step_input.latency_history.rend();
           ++it) {
        const rclcpp::Time end_of_current_step =
          it->timestamp + (step_input.timestamp_meaning == TimestampMeaning::start
                             ? to_duration(it->latency_ms)
                             : to_duration(0.0));
        if (is_timestamp_older(end_of_current_step, start_of_next_step)) {
          total_latency_ms_ += it->latency_ms;
          start_of_next_step = it->timestamp;
          debug_ss << " + " << step_it->name << "=" << it->latency_ms;
          if (step_input.timestamp_meaning == TimestampMeaning::end) {
            start_of_next_step -= to_duration(it->latency_ms);
          }
          found_valid_data = true;
          break;
        }
      }
    }
    // skip step if no valid data found
    if (!found_valid_data) {
      debug_ss << " + " << step_it->name << "=skipped";
    }
  }
  RCLCPP_DEBUG(
    get_logger(), "Total latency calculation (cumulative time-ordered): %s = %2.2fms",
    debug_ss.str().c_str(), total_latency_ms_);

  std::stringstream ss;
  ss << "offsets added to the total: [ ";
  for (const auto offset : latency_offsets_) {
    ss << offset << " ";
    total_latency_ms_ += offset;
  }
  ss << "]";

  RCLCPP_DEBUG(
    get_logger(), "Total latency with offsets: %.2f ms (%s)", total_latency_ms_, ss.str().c_str());
}

void PipelineLatencyMonitorNode::publish_total_latency()
{
  // Publish total latency
  auto total_latency_msg = std::make_unique<autoware_internal_debug_msgs::msg::Float64Stamped>();
  total_latency_msg->stamp = now();
  total_latency_msg->data = total_latency_ms_;
  total_latency_pub_->publish(std::move(total_latency_msg));

  // Publish latest latency values from each topic as debug information
  for (const auto & input : input_sequence_) {
    const auto latest_latency = get_latest_value(input.latency_history);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/" + input.name + "_latency_ms", latest_latency);
  }
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/pipeline_total_latency_ms", total_latency_ms_);

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000, "Total latency: %.2f ms (threshold: %.2f ms)",
    total_latency_ms_, latency_threshold_ms_);
}

void PipelineLatencyMonitorNode::check_total_latency(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("Total Latency (ms)", total_latency_ms_);
  stat.add("Threshold (ms)", latency_threshold_ms_);

  std::string uninitialized_inputs;
  for (const auto & input : input_sequence_) {
    if (!has_valid_data(input.latency_history)) {
      if (!uninitialized_inputs.empty()) {
        uninitialized_inputs += ", ";
      }
      uninitialized_inputs += input.name;
    } else {
      const auto latest_latency = get_latest_value(input.latency_history);
      stat.add(input.name, latest_latency);
    }
  }
  if (!uninitialized_inputs.empty()) {
    stat.add("uninitialized_inputs", uninitialized_inputs);
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Some latency inputs not yet received: " + uninitialized_inputs);
  } else if (total_latency_ms_ > latency_threshold_ms_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Total latency exceeds threshold");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK, "Total latency within acceptable range");
  }
}

void PipelineLatencyMonitorNode::update_history(
  std::deque<ProcessData> & history, const rclcpp::Time & timestamp, double value)
{
  if (value < 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 30000,
      "Negative latency value detected: %.6f, treating as 0.0", value);
    value = 0.0;
  }

  // Add new value to history
  history.emplace_back(timestamp, value);

  // Remove old data if window size is exceeded
  while (history.size() > window_size_) {
    history.pop_front();
  }
}

bool PipelineLatencyMonitorNode::is_timestamp_older(
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

}  // namespace autoware::system::pipeline_latency_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::system::pipeline_latency_monitor::PipelineLatencyMonitorNode)
