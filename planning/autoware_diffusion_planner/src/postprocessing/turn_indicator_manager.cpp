// Copyright 2025 TIER IV, Inc.
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

#include "autoware/diffusion_planner/postprocessing/turn_indicator_manager.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{
TurnIndicatorManager::TurnIndicatorManager(
  const rclcpp::Duration & hold_duration, const float keep_offset)
: hold_duration_(hold_duration), keep_offset_(keep_offset)
{
}

void TurnIndicatorManager::set_hold_duration(const rclcpp::Duration & hold_duration)
{
  hold_duration_ = hold_duration;
}

void TurnIndicatorManager::set_keep_offset(const float keep_offset)
{
  keep_offset_ = keep_offset;
}

TurnIndicatorsCommand TurnIndicatorManager::evaluate(
  std::vector<float> turn_indicator_logit, const rclcpp::Time & stamp, const int64_t prev_report)
{
  TurnIndicatorsCommand command_msg;
  command_msg.stamp = stamp;

  if (turn_indicator_logit.empty()) {
    command_msg.command = TurnIndicatorsCommand::DISABLE;
    return command_msg;
  }

  if (last_non_keep_stamp_.nanoseconds() > 0) {
    const auto expiration = last_non_keep_stamp_ + hold_duration_;
    if (stamp <= expiration) {
      command_msg.command = last_non_keep_command_;
      return command_msg;
    }
  }

  turn_indicator_logit[TURN_INDICATOR_OUTPUT_KEEP] += keep_offset_;

  const float max_logit =
    *std::max_element(turn_indicator_logit.begin(), turn_indicator_logit.end());

  std::vector<float> probabilities(turn_indicator_logit.size());
  float sum = 0.0001f;  // small constant to avoid division by zero
  for (size_t i = 0; i < turn_indicator_logit.size(); ++i) {
    probabilities[i] = std::exp(turn_indicator_logit[i] - max_logit);
    sum += probabilities[i];
  }

  for (float & prob : probabilities) {
    prob /= sum;
  }

  const size_t max_idx = std::distance(
    probabilities.begin(), std::max_element(probabilities.begin(), probabilities.end()));
  const bool keep_selected = (max_idx == TURN_INDICATOR_OUTPUT_KEEP);
  const uint8_t predicted_command =
    keep_selected ? static_cast<uint8_t>(prev_report) : static_cast<uint8_t>(max_idx);
  command_msg.command = predicted_command;

  if (!keep_selected) {
    last_non_keep_command_ = command_msg.command;
    last_non_keep_stamp_ = stamp;
  }

  return command_msg;
}

}  // namespace autoware::diffusion_planner::postprocess
