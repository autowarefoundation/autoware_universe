// Copyright 2021 TIER IV, Inc.
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

#include "autoware/map_based_prediction/map_based_prediction_node/diagnostics.hpp"

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <memory>
#include <sstream>
#include <utility>

namespace autoware::map_based_prediction
{

Diagnostics::Diagnostics(rclcpp::Node * node)
: diagnostics_interface_ptr_(
    std::make_unique<autoware_utils::DiagnosticsInterface>(node, "map_based_prediction"))
{
}

void Diagnostics::setParams(const Params & params)
{
  params_ = params;
}

void Diagnostics::setPublishedTimePublisher(
  std::unique_ptr<autoware_utils::PublishedTimePublisher> publisher)
{
  published_time_publisher_ = std::move(publisher);
}

void Diagnostics::setProcessingTimePublisher(
  std::unique_ptr<autoware_utils::DebugPublisher> publisher)
{
  processing_time_publisher_ = std::move(publisher);
}

void Diagnostics::update(
  const rclcpp::Time & timestamp, double processing_time_ms, double cyclic_time_ms)
{
  diagnostics_interface_ptr_->clear();
  diagnostics_interface_ptr_->add_key_value("timestamp", timestamp.seconds());
  diagnostics_interface_ptr_->add_key_value("processing_time_ms", processing_time_ms);

  const bool is_processing_in_time = processing_time_ms <= params_.processing_time_tolerance_ms;
  diagnostics_interface_ptr_->add_key_value("is_processing_in_time", is_processing_in_time);
  if (!is_processing_in_time) {
    std::ostringstream oss;
    oss << "Processing time exceeded: " << params_.processing_time_tolerance_ms << "[ms] < "
        << processing_time_ms << "[ms]";
    diagnostics_interface_ptr_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, oss.str());
  }

  if (is_processing_in_time || !last_in_time_processing_timestamp_) {
    last_in_time_processing_timestamp_ = timestamp;
  }

  const double consecutive_excess_duration_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds(
        (timestamp - last_in_time_processing_timestamp_.value()).nanoseconds()))
      .count();

  const bool is_consecutive_excess_duration_ok =
    consecutive_excess_duration_ms < params_.processing_time_consecutive_excess_tolerance_ms;
  diagnostics_interface_ptr_->add_key_value(
    "consecutive_excess_duration_ms", consecutive_excess_duration_ms);
  diagnostics_interface_ptr_->add_key_value(
    "is_consecutive_excess_duration_ok", is_consecutive_excess_duration_ok);
  if (!is_consecutive_excess_duration_ok) {
    std::ostringstream oss;
    oss << "Processing time exceeded consecutively in a long term: "
        << params_.processing_time_consecutive_excess_tolerance_ms << "[ms] < "
        << consecutive_excess_duration_ms << "[ms]";
    diagnostics_interface_ptr_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, oss.str());
  }

  diagnostics_interface_ptr_->publish(timestamp);

  if (processing_time_publisher_) {
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

}  // namespace autoware::map_based_prediction
