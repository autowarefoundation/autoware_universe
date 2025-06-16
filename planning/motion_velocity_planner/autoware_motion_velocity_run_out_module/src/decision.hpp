// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef DECISION_HPP_
#define DECISION_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <rclcpp/time.hpp>

#include <cstdio>
#include <optional>
#include <sstream>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief update a decision based on a priority stop > slowdown > nothing
void update_decision(
  std::optional<Decision> & decision_to_update, std::stringstream & explanation_to_update,
  const Decision & decision, const std::stringstream & explanation);
/// @brief return true if the given decision is for a detected collision
bool is_collision(const Decision & d);
/// @brief calculate the consecutive time with collisions in the history
/// @warning assumes the current time has a collision
double calculate_consecutive_time_with_collision(
  const DecisionHistory & history, const rclcpp::Time & current_time);
/// @brief return true if the conditions to stop are met
bool condition_to_stop(
  const DecisionHistory & history, const std::optional<Collision> & current_collision,
  const rclcpp::Time & current_time, std::stringstream & explanation, const Parameters & params);
/// @brief return true if the conditions to keep the previous stop decision are met
bool condition_to_keep_stop(
  const DecisionHistory & history, const Collision & collision,
  const double keep_stop_distance_range, std::stringstream & explanation,
  const rclcpp::Time & current_time, const Parameters & params);
/// @brief return true if the conditions to slowdown are met
bool condition_to_slowdown(
  const DecisionHistory & history, const std::optional<Collision> & current_collision,
  const rclcpp::Time & current_time, std::stringstream & explanation, const Parameters & params);
/// @brief return true if the conditions to keep the previous slowdown decision are met
bool condition_to_keep_slowdown(
  const DecisionHistory & history, const rclcpp::Time & current_time,
  std::stringstream & explanation, const Parameters & params);
/// @brief calculate the decision type corresponding to a collision type and a decision history
DecisionType calculate_decision_type(
  const std::optional<Collision> & collision, const DecisionHistory & history,
  const rclcpp::Time & now, std::stringstream & explanation, const double keep_stop_distance_range,
  const Parameters & params);
/// @brief update objects that did not have a decision at the given time
void update_objects_without_decisions(
  ObjectDecisionsTracker & decisions_tracker, const rclcpp::Time & now);
/// @brief calculate current decisions for the objects and update the decision tracker accordingly
void calculate_decisions(
  ObjectDecisionsTracker & decisions_tracker, const std::vector<Object> & objects,
  const rclcpp::Time & now, const double keep_stop_distance_range, const Parameters & params);
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // DECISION_HPP_
