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

#ifndef AUTOWARE__SIMPL__CONVERSION__TRACKED_OBJECT_HPP_
#define AUTOWARE__SIMPL__CONVERSION__TRACKED_OBJECT_HPP_

#include "autoware/simpl/archetype/agent.hpp"

#include <autoware_perception_msgs/msg/tracked_object.hpp>

namespace autoware::simpl::conversion
{
/**
 * @brief Convert `TrackedObject` to `AgentState`.
 *
 * @param object Tracked object.
 * @param is_valid Indicates whether this state is valid.
 */
archetype::AgentState to_state(
  const autoware_perception_msgs::msg::TrackedObject & object, bool is_valid);
}  // namespace autoware::simpl::conversion
#endif  // AUTOWARE__SIMPL__CONVERSION__TRACKED_OBJECT_HPP_
