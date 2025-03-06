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
#include "autoware/simpl/archetype/map.hpp"

#include <cmath>
#include <tuple>

namespace autoware::simpl::processing
{
/**
 * @brief Transform an agent state, which is `from`, to the coordinate frame of `to`.
 *
 * @param from Original agent state.
 * @param to Agent state coordinate where `from` will be transformed.
 */
inline archetype::AgentState transform2d(
  const archetype::AgentState & from, const archetype::AgentState & to)
{
  auto vcos = std::cos(to.yaw);
  auto vsin = std::sin(to.yaw);

  auto x = (from.x - to.x) * vcos - (from.y - to.y) * vsin;
  auto y = (from.x - to.x) * vsin + (from.y - to.y) * vcos;
  auto yaw = from.yaw - to.yaw;
  auto vx = from.vx * vcos - from.vy * vsin;
  auto vy = from.vx * vsin + from.vy * vcos;

  return {x, y, from.z, yaw, vx, vy, from.label, from.is_valid};
}

/**
 * @brief Transform state from local frame to global frame.
 *
 * @param from_x X location w.r.t local frame.
 * @param from_y Y location w.r.t local frame.
 * @param from_vx X direction velocity w.r.t local frame.
 * @param from_vy Y direction velocity w.r.t local frame.
 * @param to Agent state coordinate where `from` will be transformed.
 */
inline std::tuple<double, double, double, double> transform2d(
  double from_x, double from_y, double from_vx, double from_vy, const archetype::AgentState & to)
{
  auto vcos = std::cos(to.yaw);
  auto vsin = std::sin(to.yaw);

  double x = from_x * vcos - from_y * vsin + to.x;
  double y = from_x * vsin + from_y * vcos + to.y;
  double vx = from_vx * vcos - from_vy * vsin;
  double vy = from_vx * vsin + from_vy * vcos;

  return {x, y, vx, vy};
}

/**
 * @brief Transform a map point, which is `from`, to the coordinate frame of `to`.
 *
 * @param from Original map point.
 * @param to Agent state coordinate where `from` will be transformed.
 */
inline archetype::MapPoint transform2d(
  const archetype::MapPoint & from, const archetype::AgentState & to)
{
  auto vcos = std::cos(to.yaw);
  auto vsin = std::sin(to.yaw);

  auto x = (from.x - to.x) * vcos - (from.y - to.y) * vsin;
  auto y = (from.x - to.x) * vsin + (from.y - to.y) * vcos;
  auto dx = from.dx * vcos - from.dy * vsin;
  auto dy = from.dx * vsin + from.dy * vcos;

  return {x, y, from.z, dx, dy, from.dz, from.label};
}

/**
 * @brief Transform a map point, which is `from`, to the coordinate frame of the specified pose.
 *
 * @param from Original map point.
 * @param to_x X location w.r.t other coordinate.
 * @param to_y Y location w.r.t other coordinate.
 * @param to_yaw Yaw angle w.r.t other coordinate.
 * @return archetype::MapPoint
 */
inline archetype::MapPoint transform2d(
  const archetype::MapPoint & from, double to_x, double to_y, double to_yaw)
{
  auto vcos = std::cos(to_yaw);
  auto vsin = std::sin(to_yaw);

  auto x = (from.x - to_x) * vcos - (from.y - to_y) * vsin;
  auto y = (from.x - to_x) * vsin + (from.y - to_y) * vcos;
  auto dx = from.dx * vcos - from.dy * vsin;
  auto dy = from.dx * vsin + from.dy * vcos;

  return {x, y, from.z, dx, dy, from.dz, from.label};
}
}  // namespace autoware::simpl::processing
#endif  // AUTOWARE__SIMPL__PROCESSING__GEOMETRY_HPP_
