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

#ifndef AUTOWARE__SIMPL__PROCESSING__GEOMETRY_HPP_
#define AUTOWARE__SIMPL__PROCESSING__GEOMETRY_HPP_

#include "autoware/simpl/archetype/agent.hpp"

#include <cmath>

namespace autoware::simpl::processing
{
/**
 * @brief Transform an agent state, which is `from`, to the coordinate frame of `to`.
 *
 * @param from Original agent state.
 * @param to Agent state coordinate where `from` will be transformed.
 */
archetype::AgentState transform_2d(
  const archetype::AgentState & from, const archetype::AgentState & to)
{
  auto cos = std::cos(to.yaw);
  auto sin = std::sin(to.yaw);

  auto ret_x = (from.x - to.x) * cos - (from.y - to.y) * sin;
  auto ret_y = (from.x - to.x) * sin + (from.y - to.y) * cos;
  auto ret_yaw = from.yaw - to.yaw;
  auto ret_vx = from.vx * cos - from.vy * sin;
  auto ret_vy = from.vx * sin + from.vy * cos;

  return {ret_x, ret_y, from.z, ret_yaw, ret_vx, ret_vy, from.label, from.is_valid};
}

/**
 * @brief Transform a map point, which is `from`, to the coordinate frame of `to`.
 *
 * @param from Original map point.
 * @param to Agent state coordinate where `from` will be transformed.
 */
archetype::MapPoint transform_2d(const archetype::MapPoint & from, const archetype::AgentState & to)
{
  auto cos_v = std::cos(to.yaw);
  auto sin = std::sin(to.yaw);

  auto ret_x = (from.x - to.x) * cos - (from.y - to.y) * sin;
  auto ret_y = (from.x - to.x) * sin + (from.y - to.y) * cos;
  auto ret_dx = from.dx * cos - from.dy * sin;
  auto ret_dy = from.dx * sin + from.dy * cos;

  return {ret_x, ret_y, from.z, ret_dx, ret_dy, from.dz, from.label};
}

/**
 * @brief Transform a map point, which is `from`, to the coordinate frame of the specified pose.
 *
 * @param from
 * @param to_x
 * @param to_y
 * @param to_yaw
 * @return archetype::MapPoint
 */
archetype::MapPoint transform_2d(
  const archetype::MapPoint & from, double to_x, double to_y, double to_yaw)
{
  auto cos = std::cos(to_yaw);
  auto sin = std::sin(to_yaw);

  auto ret_x = (from.x - to_x) * cos - (from.y - to_y) * sin;
  auto ret_y = (from.x - to_x) * sin + (from.y - to_y) * cos;
  auto ret_dx = from.dx * cos - from.dy * sin;
  auto ret_dy = from.dx * sin + from.dy * cos;

  return {ret_x, ret_y, from.z, ret_dx, ret_dy, from.dz, from.label};
}
}  // namespace autoware::simpl::processing
#endif  // AUTOWARE__SIMPL__PROCESSING__GEOMETRY_HPP_
