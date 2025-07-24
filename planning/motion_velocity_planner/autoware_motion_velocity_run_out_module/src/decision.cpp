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

#include "decision.hpp"

#include "parameters.hpp"
#include "types.hpp"

#include <algorithm>
#include <cstdio>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
void update_decision(
  std::optional<Decision> & decision_to_update, std::stringstream & explanation_to_update,
  const Decision & decision, const std::stringstream & explanation)
{
  const auto not_set_yet = !decision_to_update;
  const auto same_decision_but_earlier =
    decision_to_update && decision.collision.has_value() &&
    (decision_to_update->type == decision.type &&
     decision.collision->ego_collision_time < decision_to_update->collision->ego_collision_time);
  const auto higher_priority_decision =
    decision_to_update && ((decision_to_update->type == slowdown && decision.type == stop) ||
                           (decision_to_update->type == nothing && decision.type != nothing));
  if (not_set_yet || same_decision_but_earlier || higher_priority_decision) {
    decision_to_update = decision;
    explanation_to_update = std::stringstream() << explanation.rdbuf();
  }
}

bool is_collision(const Decision & d)
{
  return d.collision.has_value() && d.collision->type == collision;
}

double calculate_consecutive_time_with_collision(
  const DecisionHistory & history, const rclcpp::Time & current_time)
{
  if (history.times.empty()) {
    return 0.0;
  }
  const auto first_not_collision_it =
    std::find_if(history.decisions.rbegin(), history.decisions.rend(), std::not_fn(is_collision));
  if (first_not_collision_it == history.decisions.rend()) {  // all collisions in history
    return current_time.seconds() - history.times.front();
  }
  const auto first_not_collision_idx =
    std::distance(history.decisions.begin(), first_not_collision_it.base()) - 1;
  const auto first_consecutive_collision_idx = first_not_collision_idx + 1UL;
  if (first_consecutive_collision_idx == history.times.size()) {
    return 0.0;
  }
  return current_time.seconds() - history.times[first_consecutive_collision_idx];
}

bool condition_to_stop(
  const DecisionHistory & history, const std::optional<Collision> & current_collision,
  const rclcpp::Time & current_time, std::stringstream & explanation, const Parameters & params)
{
  if (!current_collision) {
    return false;
  }
  // only stop after successively detecting collisions for some time
  if (current_collision->type == collision) {
    const auto previous_decision =
      history.decisions.empty() ? std::nullopt : std::make_optional(history.decisions.back().type);
    if (previous_decision == stop) {
      explanation << "stopping since finding a collision and previous decision was STOP";
      return true;
    }
    const auto consecutive_time_with_collision =
      calculate_consecutive_time_with_collision(history, current_time);
    if (consecutive_time_with_collision >= params.stop_on_time_buffer) {
      explanation << "stopping since finding collisions for " << consecutive_time_with_collision
                  << "s (" << params.stop_on_time_buffer << "s buffer)";
      return true;
    }
    explanation << "not stopping since only finding collisions for "
                << consecutive_time_with_collision << "s (" << params.stop_on_time_buffer
                << "s buffer)";
    return false;
  }
  if (current_collision->type == ignored_collision) {
    explanation << "ignoring passing collision";
  }
  return false;
}

bool condition_to_keep_stop(
  const DecisionHistory & history, const std::optional<Collision> & collision,
  const double keep_stop_distance_range, std::stringstream & explanation,
  const rclcpp::Time & current_time, const Parameters & params)
{
  if (history.decisions.empty() || history.decisions.back().type != stop) {
    return false;
  }
  if (
    collision &&
    collision->ego_time_interval.first_intersection.arc_length < keep_stop_distance_range) {
    explanation << "keep stop since found collision "
                << collision->ego_time_interval.first_intersection.arc_length << "m ahead ("
                << keep_stop_distance_range << " m range)";
    return true;
  }
  // keep stopping for some time after the last detected collision
  const auto most_recent_collision_it =
    std::find_if(history.decisions.rbegin(), history.decisions.rend(), is_collision);
  if (most_recent_collision_it == history.decisions.rend()) {
    explanation << "remove stop since no collision in history";
    return false;
  }
  // -1 because the reverse iterator has an offset compared to base iterator
  const auto i = std::distance(history.decisions.begin(), most_recent_collision_it.base()) - 1;
  const auto time_since_last_collision = current_time.seconds() - history.times[i];
  if (time_since_last_collision < params.stop_off_time_buffer) {
    explanation << "keep stop since last collision found " << time_since_last_collision << "s ago ("
                << params.stop_off_time_buffer << "s buffer)";
    return true;
  }
  explanation << "remove stop since last collision found " << time_since_last_collision << "s ago ("
              << params.stop_off_time_buffer << "s buffer)";
  return false;
}

bool condition_to_slowdown(
  const DecisionHistory & history, const std::optional<Collision> & current_collision,
  const rclcpp::Time & current_time, std::stringstream & explanation, const Parameters & params)
{
  // only slowdown after successively detecting collisions for some time
  if (!current_collision || current_collision->type != collision) {
    return false;
  }
  const auto previous_decision =
    history.decisions.empty() ? std::nullopt : std::make_optional(history.decisions.back().type);
  if (previous_decision == slowdown) {
    explanation << "slowdown since finding a collision and previous decision was SLOWDOWN";
    return true;
  }
  const auto consecutive_time_with_collision =
    calculate_consecutive_time_with_collision(history, current_time);
  if (consecutive_time_with_collision >= params.slowdown_on_time_buffer) {
    explanation << "preventive slowdown since finding collisions for "
                << consecutive_time_with_collision << "s (" << params.slowdown_on_time_buffer
                << "s buffer)";
    return true;
  }
  explanation << "no slowdown since only finding collisions for " << consecutive_time_with_collision
              << "s (" << params.slowdown_on_time_buffer << "s buffer)";
  return false;
}

bool condition_to_keep_slowdown(
  const DecisionHistory & history, const rclcpp::Time & current_time,
  std::stringstream & explanation, const Parameters & params)
{
  // keep decision for some time after the last detected collision
  if (history.decisions.empty() || history.decisions.back().type != slowdown) {
    return false;
  }
  const auto most_recent_collision_it =
    std::find_if(history.decisions.rbegin(), history.decisions.rend(), is_collision);
  if (most_recent_collision_it == history.decisions.rend()) {
    explanation << "remove slowdown since no collision in history";
    return false;
  }
  // -1 because the reverse iterator has an offset compared to base iterator
  const auto i = std::distance(history.decisions.begin(), most_recent_collision_it.base()) - 1;
  const auto time_since_last_collision = current_time.seconds() - history.times[i];
  if (time_since_last_collision < params.slowdown_off_time_buffer) {
    explanation << "keep slowdown since last collision found " << time_since_last_collision
                << "s ago (" << params.slowdown_off_time_buffer << "s buffer)"
                << most_recent_collision_it->type;
    return true;
  }
  explanation << "remove slowdown since last collision found " << time_since_last_collision
              << "s ago (" << params.slowdown_off_time_buffer << "s buffer)";
  return false;
}

DecisionType calculate_decision_type(
  const std::optional<Collision> & collision, const DecisionHistory & history,
  const rclcpp::Time & now, std::stringstream & explanation, const double keep_stop_distance_range,
  const Parameters & params)
{
  if (
    condition_to_stop(history, collision, now, explanation, params) ||
    condition_to_keep_stop(
      history, collision, keep_stop_distance_range, explanation, now, params)) {
    return stop;
  }
  if (
    condition_to_slowdown(history, collision, now, explanation, params) ||
    condition_to_keep_slowdown(history, now, explanation, params)) {
    return slowdown;
  }
  return nothing;
}

void update_objects_without_decisions(
  ObjectDecisionsTracker & decisions_tracker, const rclcpp::Time & now)
{
  for (auto & [_, history] : decisions_tracker.history_per_object) {
    if (!history.times.empty() && history.times.back() != now.seconds()) {
      history.times.push_back(now.seconds());
      history.decisions.emplace_back();
    }
  }
  // remove histories where all decisions are "nothing"
  constexpr auto nothing_and_no_collision = [](const Decision & decision) {
    return decision.type == nothing &&
           (!decision.collision.has_value() || decision.collision->type == no_collision);
  };
  for (auto it = decisions_tracker.history_per_object.begin();
       it != decisions_tracker.history_per_object.end();) {
    const auto & history = it->second;
    if (std::all_of(history.decisions.begin(), history.decisions.end(), nothing_and_no_collision)) {
      it = decisions_tracker.history_per_object.erase(it);
    } else {
      ++it;
    }
  }
}
std::unordered_map<std::string, autoware_internal_planning_msgs::msg::SafetyFactor>
calculate_decisions(
  ObjectDecisionsTracker & decisions_tracker, const std::vector<Object> & objects,
  const rclcpp::Time & now, const double keep_stop_distance_range, const Parameters & params)
{
  std::unordered_map<std::string, autoware_internal_planning_msgs::msg::SafetyFactor>
    safety_factor_per_object;
  for (const auto & object : objects) {
    autoware_internal_planning_msgs::msg::SafetyFactor safety_factor;
    safety_factor.object_id = object.object->predicted_object.object_id;
    std::optional<Decision> object_decision;
    auto & decision_history = decisions_tracker.history_per_object[object.uuid];
    std::stringstream object_decision_explanation;
    for (const auto & collision : object.collisions) {
      std::stringstream explanation;
      const auto decision_type = calculate_decision_type(
        collision, decision_history, now, explanation, keep_stop_distance_range, params);
      Decision d(collision, decision_type);
      update_decision(object_decision, object_decision_explanation, d, explanation);
    }
    if (object_decision) {
      object_decision->explanation = object_decision_explanation.str();
      decision_history.add_decision(now.seconds(), *object_decision);

      if (object_decision->type != nothing && object_decision->collision) {
        safety_factor.is_safe = false;
        safety_factor.ttc_begin = static_cast<float>(std::abs(
          object_decision->collision->ego_time_interval.first_intersection.ego_time -
          object_decision->collision->ego_time_interval.first_intersection.object_time));
        safety_factor.ttc_end = static_cast<float>(std::abs(
          object_decision->collision->ego_time_interval.last_intersection.ego_time -
          object_decision->collision->ego_time_interval.last_intersection.object_time));
        geometry_msgs::msg::Point p;
        p.x = object_decision->collision->ego_time_interval.first_intersection.intersection.x();
        p.y = object_decision->collision->ego_time_interval.first_intersection.intersection.y();
        safety_factor.points.push_back(p);
        safety_factor_per_object.emplace(object.uuid, safety_factor);
      }
    }
  }
  // handle objects in the history which did not get updated this iteration
  for (auto & [_, history] : decisions_tracker.history_per_object) {
    if (!history.times.empty() && history.times.back() != now.seconds()) {
      Decision d;
      std::stringstream explanation;
      d.type = calculate_decision_type(
        std::nullopt, history, now, explanation, keep_stop_distance_range, params);
      history.add_decision(now.seconds(), d);
    }
    history.remove_outdated(now, params.max_history_duration);
  }
  update_objects_without_decisions(decisions_tracker, now);
  return safety_factor_per_object;
}
}  // namespace autoware::motion_velocity_planner::run_out
