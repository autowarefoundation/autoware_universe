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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_BLIND_SPOT_MODULE__PARAMETER_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_BLIND_SPOT_MODULE__PARAMETER_HPP_

#include <rclcpp/node.hpp>

#include <string>

namespace autoware::behavior_velocity_planner
{
struct PlannerParam
{
  static PlannerParam init(rclcpp::Node & node, const std::string & ns);
  double backward_attention_length{};
  double ttc_start_margin{};
  double ttc_end_margin{};
  double minimum_default_velocity{};
  double collision_judge_debounce{};
  double critical_stopline_margin{};
  struct Brake
  {
    struct Critical
    {
      double deceleration{};
      double jerk{};
    } critical;
    struct SemiCritical
    {
      double deceleration{};
      double jerk{};
    } semi_critical;
  } brake;
  struct BrakeForTTC
  {
    double critical_threshold_ub{};
    double semi_critical_threshold_lb{};
  } brake_for_ttc;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_BLIND_SPOT_MODULE__PARAMETER_HPP_
