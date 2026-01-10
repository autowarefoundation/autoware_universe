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

class TurnIndicatorManager
{
public:
  explicit TurnIndicatorManager(const rclcpp::Duration & hold_duration);

  TurnIndicatorsCommand evaluate(
    std::vector<float> turn_indicator_logit, const rclcpp::Time & stamp, int64_t prev_report,
    float keep_offset);

  void set_hold_duration(const rclcpp::Duration & hold_duration);

private:
  rclcpp::Duration hold_duration_;
  uint8_t last_non_keep_command_{TurnIndicatorsCommand::DISABLE};
  rclcpp::Time last_non_keep_stamp_{};
};

}  // namespace autoware::diffusion_planner::postprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__TURN_INDICATOR_MANAGER_HPP_
