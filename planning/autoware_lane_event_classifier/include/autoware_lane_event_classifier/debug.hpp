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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__DEBUG_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__DEBUG_HPP_

#include <autoware_lane_event_classifier/lane_event_classifier_base.hpp>
#include <autoware_lane_event_classifier/lane_following/checker.hpp>
#include <autoware_lane_event_classifier/types.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::lane_event_classifier
{

/** @brief Owns the node's observability: RViz markers, processing-time messages, and logging. */
class LaneEventClassifierDebug
{
public:
  explicit LaneEventClassifierDebug(rclcpp::Node & node);

  /**
   * @brief Publishes the collected classifier markers (no-op if empty).
   * @param markers Marker array to publish.
   */
  void publish_markers(const visualization_msgs::msg::MarkerArray & markers) const;

  /**
   * @brief Logs a throttled warning (e.g. a cycle skipped because an input was unavailable).
   * @param message Human-readable warning message.
   */
  void log_warn(const std::string & message) const;

  /**
   * @brief Logs state transitions and accumulating departures.
   * @param current_state Latest published DrivingState.
   * @param input Per-cycle input.
   * @param lane_following_result Lane-following gate verdict for the cycle.
   * @param classifiers Active classifiers, for their debug reasons.
   */
  void log_state(
    uint8_t current_state, const LaneEventInput & input,
    const LaneFollowingResult & lane_following_result,
    const std::vector<std::unique_ptr<LaneEventClassifierBase>> & classifiers);

  /**
   * @brief Publishes the processing-time value and the per-section text overlay (running max).
   * @param stamp Message timestamp.
   * @param total_time_ms Total cycle time in milliseconds.
   * @param section_times Per-section (name, time_ms) pairs, in display order.
   */
  void publish_processing_time(
    const builtin_interfaces::msg::Time & stamp, double total_time_ms,
    const std::vector<std::pair<std::string, double>> & section_times);

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::StringStamped>::SharedPtr
    pub_processing_time_text_;

  // Running maximum per timed section, for the processing-time text overlay.
  std::unordered_map<std::string, double> max_processing_time_ms_;

  uint8_t previously_published_state_{DrivingState::UNKNOWN};  // last state, to log transitions
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__DEBUG_HPP_
