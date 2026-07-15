// Copyright 2026 TIER IV, Inc.
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

#include "autoware_lane_event_classifier/debug.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::lane_event_classifier
{

namespace
{
const char * state_to_string(uint8_t state)
{
  switch (state) {
    case DrivingState::UNKNOWN:
      return "UNKNOWN";
    case DrivingState::LANE_FOLLOWING:
      return "LANE_FOLLOWING";
    case DrivingState::LANE_CHANGING:
      return "LANE_CHANGING";
    case DrivingState::ABORTING_LANE_CHANGE:
      return "ABORTING_LANE_CHANGE";
    case DrivingState::INTENTIONAL_LANE_CROSSING:
      return "INTENTIONAL_LANE_CROSSING";
    case DrivingState::ABORTING_INTENTIONAL_LANE_CROSSING:
      return "ABORTING_INTENTIONAL_LANE_CROSSING";
    default:
      return "?";
  }
}
}  // namespace

LaneEventClassifierDebug::LaneEventClassifierDebug(rclcpp::Node & node)
: logger_{node.get_logger()}, clock_{node.get_clock()}
{
  pub_markers_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", rclcpp::QoS{1});
  pub_processing_time_ = node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", rclcpp::QoS{1});
  pub_processing_time_text_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
      "~/debug/processing_time_text", rclcpp::QoS{1});
}

void LaneEventClassifierDebug::publish_markers(
  const visualization_msgs::msg::MarkerArray & markers) const
{
  if (markers.markers.empty()) {
    return;
  }
  pub_markers_->publish(markers);
}

void LaneEventClassifierDebug::log_warn(const std::string & message) const
{
  RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "%s", message.c_str());
}

void LaneEventClassifierDebug::log_state(
  uint8_t current_state, const LaneEventInput & input,
  const LaneFollowingResult & lane_following_result,
  const std::vector<std::unique_ptr<LaneEventClassifierBase>> & classifiers)
{
  const auto & ego_pos = input.odometry_ptr->pose.pose.position;
  const auto & [is_lane_following, lane_following_reason] = lane_following_result;
  const bool ego_departed = !is_lane_following;
  const auto following_reason = to_string(lane_following_reason);

  if (current_state != previously_published_state_) {
    std::string reasons;
    for (const auto & classifier : classifiers) {
      const auto reason = classifier->debug_reason();
      if (!reason.empty()) {
        reasons += reasons.empty() ? reason : "; " + reason;
      }
    }
    RCLCPP_INFO(
      logger_, "%s",
      fmt::format(
        "[lane_event] {} -> {} | ego=({:.2f}, {:.2f}) following={} ({}) | why: {}",
        state_to_string(previously_published_state_), state_to_string(current_state), ego_pos.x,
        ego_pos.y, is_lane_following, following_reason, reasons.empty() ? "(none)" : reasons)
        .c_str());
    previously_published_state_ = current_state;
  } else if (current_state == DrivingState::LANE_FOLLOWING && ego_departed) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000, "%s",
      fmt::format(
        "[lane_event] departure accumulating: ego not lane following ({})", following_reason)
        .c_str());
  }
}

void LaneEventClassifierDebug::publish_processing_time(
  const builtin_interfaces::msg::Time & stamp, double total_time_ms,
  const std::vector<std::pair<std::string, double>> & section_times)
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = stamp;
  processing_time_msg.data = total_time_ms;
  pub_processing_time_->publish(processing_time_msg);

  // Processing-time text overlay: current value with a running max per section, e.g.
  //   total: 0.33 (max: 1.00) [ms]
  //   lane_following: 0.08 (max: 0.30) [ms]
  //   lane_change: 0.05 (max: 0.20) [ms]
  const auto format_processing_time = [this](const std::string & label, double time_ms) {
    double & max_time_ms = max_processing_time_ms_[label];
    max_time_ms = std::max(max_time_ms, time_ms);
    return fmt::format("{}: {:.2f} (max: {:.2f}) [ms]", label, time_ms, max_time_ms);
  };
  std::string processing_time_text = format_processing_time("total", total_time_ms);
  for (const auto & [section_name, section_time_ms] : section_times) {
    processing_time_text += "\n" + format_processing_time(section_name, section_time_ms);
  }
  autoware_internal_debug_msgs::msg::StringStamped processing_time_text_msg;
  processing_time_text_msg.stamp = stamp;
  processing_time_text_msg.data = processing_time_text;
  pub_processing_time_text_->publish(processing_time_text_msg);
}

}  // namespace autoware::lane_event_classifier
