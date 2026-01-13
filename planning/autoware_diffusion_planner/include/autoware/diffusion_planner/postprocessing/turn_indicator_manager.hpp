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

#ifndef AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__TURN_INDICATOR_MANAGER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__TURN_INDICATOR_MANAGER_HPP_

#include "autoware/diffusion_planner/dimensions.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <vector>

namespace autoware::diffusion_planner::postprocess
{
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;

/**
 * @brief Manages turn indicator decisions with keep and hold behaviors.
 */
class TurnIndicatorManager
{
public:
  /**
   * @brief Constructs a manager that holds the last non-keep turn command for a duration.
   *
   * @param hold_duration Duration to keep the last non-keep command before allowing updates.
   * @param keep_offset Bias added to the keep logit.
   */
  explicit TurnIndicatorManager(const rclcpp::Duration & hold_duration, float keep_offset);

  /**
   * @brief Evaluates turn indicator logits into a command with hold/keep logic.
   *
   * @param turn_indicator_logit Logits for turn indicator classes.
   * @param stamp Timestamp for the command message.
   * @param prev_report Previous turn indicator report (used when keep is selected).
   * @return TurnIndicatorsCommand with the selected command and stamp.
   */
  TurnIndicatorsCommand evaluate(
    std::vector<float> turn_indicator_logit, const rclcpp::Time & stamp, const int64_t prev_report);

  /**
   * @brief Updates the hold duration for the last non-keep command.
   *
   * @param hold_duration New duration to keep the last non-keep command.
   */
  void set_hold_duration(const rclcpp::Duration & hold_duration);

  /**
   * @brief Updates the keep logit bias.
   *
   * @param keep_offset Bias added to the keep logit.
   */
  void set_keep_offset(float keep_offset);

private:
  rclcpp::Duration hold_duration_;
  float keep_offset_{0.0f};
  uint8_t last_non_keep_command_{TurnIndicatorsCommand::DISABLE};
  rclcpp::Time last_non_keep_stamp_{};
};

}  // namespace autoware::diffusion_planner::postprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__TURN_INDICATOR_MANAGER_HPP_
