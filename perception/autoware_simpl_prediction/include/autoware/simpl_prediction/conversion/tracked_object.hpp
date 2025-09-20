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

#ifndef AUTOWARE__SIMPL_PREDICTION__CONVERSION__TRACKED_OBJECT_HPP_
#define AUTOWARE__SIMPL_PREDICTION__CONVERSION__TRACKED_OBJECT_HPP_

#include "autoware/simpl_prediction/archetype/agent.hpp"

#include <autoware_perception_msgs/msg/detail/tracked_object__struct.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::simpl_prediction::conversion
{
/**
 *
 */
archetype::AgentLabel to_agent_label(const autoware_perception_msgs::msg::TrackedObject & object);

/**
 * @brief Convert Autoware `TrackedObject` message to internal `AgentState`.
 *
 * Assumes pose is in map coordinate frame, but velocity is in object coordinate frame.
 *
 * @param object Tracked object.
 */
archetype::AgentState to_agent_state(const autoware_perception_msgs::msg::TrackedObject & object);

/**
 * @brief Convert ROS 2 `Odometry` message of the ego to internal `AgentState`.
 *
 * Assumes pose is in map coordinate frame, but velocity is in object coordinate frame.
 *
 * @param odometry Ego odometry.
 * @param is_valid Indicates whether this state is valid.
 */
archetype::AgentState to_agent_state(const nav_msgs::msg::Odometry & odometry);
}  // namespace autoware::simpl_prediction::conversion
#endif  // AUTOWARE__SIMPL_PREDICTION__CONVERSION__TRACKED_OBJECT_HPP_
