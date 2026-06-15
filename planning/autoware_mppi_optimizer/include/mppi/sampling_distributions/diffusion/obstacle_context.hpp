// Copyright 2026 TIER IV, Inc.
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

/**
 * Fixed-size ego-relative obstacle encoding for diffusion sampler conditioning.
 *
 * Layout (kDiffusionContextDim = 35):
 *   Per obstacle slot (up to 4): dx, dy, cos(d_yaw), sin(d_yaw), half_length, half_width, vx_ego,
 * vy_ego Scene tail: boundary_left, boundary_right, ego_vel
 *
 * Positions/velocities are in the ego rear-axle frame at MPPI decision time. Unused obstacle slots
 * are zero.
 */
#pragma once

#ifndef MPPI__SAMPLING_DISTRIBUTIONS__DIFFUSION__OBSTACLE_CONTEXT_HPP_
#define MPPI__SAMPLING_DISTRIBUTIONS__DIFFUSION__OBSTACLE_CONTEXT_HPP_

#include <mppi/cost_functions/moving_car_obstacles.hpp>
#include <mppi/cost_functions/parked_car_obstacles.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace mppi
{
namespace sampling_distributions
{
namespace diffusion
{

constexpr int kMaxContextObstacles = 4;
constexpr int kObstacleFeatureDim = 8;
constexpr int kSceneContextDim = 3;
constexpr int kDiffusionContextDim = kMaxContextObstacles * kObstacleFeatureDim + kSceneContextDim;

inline void zeroDiffusionContext(float * out)
{
  std::memset(out, 0, sizeof(float) * static_cast<size_t>(kDiffusionContextDim));
}

inline void encodeObstacleSlot(
  const float ego_x, const float ego_y, const float ego_cos, const float ego_sin, const float obs_x,
  const float obs_y, const float obs_yaw, const float obs_half_length, const float obs_half_width,
  const float obs_vx_world, const float obs_vy_world, float * slot_out)
{
  const float dx_world = obs_x - ego_x;
  const float dy_world = obs_y - ego_y;
  slot_out[0] = ego_cos * dx_world + ego_sin * dy_world;
  slot_out[1] = -ego_sin * dx_world + ego_cos * dy_world;
  const float d_yaw = obs_yaw - std::atan2(ego_sin, ego_cos);
  slot_out[2] = std::cos(d_yaw);
  slot_out[3] = std::sin(d_yaw);
  slot_out[4] = obs_half_length;
  slot_out[5] = obs_half_width;
  slot_out[6] = ego_cos * obs_vx_world + ego_sin * obs_vy_world;
  slot_out[7] = -ego_sin * obs_vx_world + ego_cos * obs_vy_world;
}

/** Encode nearest obstacles by squared distance (static or moving at decision time). */
inline void encodeDiffusionObstacleContext(
  const float ego_x, const float ego_y, const float ego_yaw, const float ego_vel,
  const float boundary_left, const float boundary_right, const float * obs_x, const float * obs_y,
  const float * obs_yaw, const float * obs_half_length, const float * obs_half_width,
  const float * obs_vx_world, const float * obs_vy_world, const int num_obstacles, float * out)
{
  zeroDiffusionContext(out);
  const float ego_cos = std::cos(ego_yaw);
  const float ego_sin = std::sin(ego_yaw);

  struct RankedObstacle
  {
    int index = 0;
    float dist_sq = 0.0F;
  };

  RankedObstacle ranked[kMaxContextObstacles];
  const int n = std::max(0, num_obstacles);
  const int keep = std::min(n, kMaxContextObstacles);
  for (int i = 0; i < keep; ++i) {
    const float dx = obs_x[i] - ego_x;
    const float dy = obs_y[i] - ego_y;
    ranked[i].index = i;
    ranked[i].dist_sq = dx * dx + dy * dy;
  }
  std::sort(ranked, ranked + keep, [](const RankedObstacle & a, const RankedObstacle & b) {
    return a.dist_sq < b.dist_sq;
  });

  for (int slot = 0; slot < keep; ++slot) {
    const int i = ranked[slot].index;
    const float vx = obs_vx_world != nullptr ? obs_vx_world[i] : 0.0F;
    const float vy = obs_vy_world != nullptr ? obs_vy_world[i] : 0.0F;
    encodeObstacleSlot(
      ego_x, ego_y, ego_cos, ego_sin, obs_x[i], obs_y[i], obs_yaw[i], obs_half_length[i],
      obs_half_width[i], vx, vy, &out[slot * kObstacleFeatureDim]);
  }

  const int scene_base = kMaxContextObstacles * kObstacleFeatureDim;
  out[scene_base + 0] = boundary_left;
  out[scene_base + 1] = boundary_right;
  out[scene_base + 2] = ego_vel;
}

namespace detail
{

inline void gatherActiveMovingCarObstacles(
  const std::vector<mppi::cost::MovingCarObstacle> & cars, const float sim_time,
  std::vector<float> & obs_x, std::vector<float> & obs_y, std::vector<float> & obs_yaw,
  std::vector<float> & obs_half_length, std::vector<float> & obs_half_width,
  std::vector<float> & obs_vx, std::vector<float> & obs_vy)
{
  obs_x.clear();
  obs_y.clear();
  obs_yaw.clear();
  obs_half_length.clear();
  obs_half_width.clear();
  obs_vx.clear();
  obs_vy.clear();
  obs_x.reserve(cars.size());
  obs_y.reserve(cars.size());
  obs_yaw.reserve(cars.size());
  obs_half_length.reserve(cars.size());
  obs_half_width.reserve(cars.size());
  obs_vx.reserve(cars.size());
  obs_vy.reserve(cars.size());
  for (const mppi::cost::MovingCarObstacle & car : cars) {
    if (!car.isActiveAt(sim_time)) {
      continue;
    }
    const mppi::cost::ParkedCarObstacle pose = car.poseAt(sim_time);
    obs_x.push_back(pose.ox);
    obs_y.push_back(pose.oy);
    obs_yaw.push_back(pose.yaw);
    obs_half_length.push_back(pose.length * 0.5F);
    obs_half_width.push_back(pose.width * 0.5F);
    obs_vx.push_back(car.vx);
    obs_vy.push_back(car.vy);
  }
}

inline void gatherParkedCarObstacles(
  const std::vector<mppi::cost::ParkedCarObstacle> & cars, std::vector<float> & obs_x,
  std::vector<float> & obs_y, std::vector<float> & obs_yaw, std::vector<float> & obs_half_length,
  std::vector<float> & obs_half_width)
{
  obs_x.clear();
  obs_y.clear();
  obs_yaw.clear();
  obs_half_length.clear();
  obs_half_width.clear();
  obs_x.reserve(cars.size());
  obs_y.reserve(cars.size());
  obs_yaw.reserve(cars.size());
  obs_half_length.reserve(cars.size());
  obs_half_width.reserve(cars.size());
  for (const mppi::cost::ParkedCarObstacle & car : cars) {
    obs_x.push_back(car.ox);
    obs_y.push_back(car.oy);
    obs_yaw.push_back(car.yaw);
    obs_half_length.push_back(car.length * 0.5F);
    obs_half_width.push_back(car.width * 0.5F);
  }
}

}  // namespace detail

/** Build the fixed-size diffusion context vector from active moving obstacles at sim_time. */
inline std::vector<float> encodeDiffusionObstacleContextFromMovingCars(
  const float ego_x, const float ego_y, const float ego_yaw, const float ego_vel,
  const float boundary_left, const float boundary_right,
  const std::vector<mppi::cost::MovingCarObstacle> & cars, const float sim_time)
{
  std::vector<float> obs_x;
  std::vector<float> obs_y;
  std::vector<float> obs_yaw;
  std::vector<float> obs_half_length;
  std::vector<float> obs_half_width;
  std::vector<float> obs_vx;
  std::vector<float> obs_vy;
  detail::gatherActiveMovingCarObstacles(
    cars, sim_time, obs_x, obs_y, obs_yaw, obs_half_length, obs_half_width, obs_vx, obs_vy);

  std::vector<float> out(static_cast<size_t>(kDiffusionContextDim), 0.0F);
  encodeDiffusionObstacleContext(
    ego_x, ego_y, ego_yaw, ego_vel, boundary_left, boundary_right, obs_x.data(), obs_y.data(),
    obs_yaw.data(), obs_half_length.data(), obs_half_width.data(), obs_vx.data(), obs_vy.data(),
    static_cast<int>(obs_x.size()), out.data());
  return out;
}

/** In-place variant for callers that reuse a pre-sized buffer (e.g. DiffusionDistribution). */
inline void encodeDiffusionObstacleContextFromMovingCars(
  const float ego_x, const float ego_y, const float ego_yaw, const float ego_vel,
  const float boundary_left, const float boundary_right,
  const std::vector<mppi::cost::MovingCarObstacle> & cars, const float sim_time, float * out)
{
  std::vector<float> obs_x;
  std::vector<float> obs_y;
  std::vector<float> obs_yaw;
  std::vector<float> obs_half_length;
  std::vector<float> obs_half_width;
  std::vector<float> obs_vx;
  std::vector<float> obs_vy;
  detail::gatherActiveMovingCarObstacles(
    cars, sim_time, obs_x, obs_y, obs_yaw, obs_half_length, obs_half_width, obs_vx, obs_vy);
  encodeDiffusionObstacleContext(
    ego_x, ego_y, ego_yaw, ego_vel, boundary_left, boundary_right, obs_x.data(), obs_y.data(),
    obs_yaw.data(), obs_half_length.data(), obs_half_width.data(), obs_vx.data(), obs_vy.data(),
    static_cast<int>(obs_x.size()), out);
}

/** Build the fixed-size diffusion context vector from static parked obstacles (zero velocity). */
inline std::vector<float> encodeDiffusionObstacleContextFromParkedCars(
  const float ego_x, const float ego_y, const float ego_yaw, const float ego_vel,
  const float boundary_left, const float boundary_right,
  const std::vector<mppi::cost::ParkedCarObstacle> & cars)
{
  std::vector<float> obs_x;
  std::vector<float> obs_y;
  std::vector<float> obs_yaw;
  std::vector<float> obs_half_length;
  std::vector<float> obs_half_width;
  detail::gatherParkedCarObstacles(cars, obs_x, obs_y, obs_yaw, obs_half_length, obs_half_width);

  std::vector<float> out(static_cast<size_t>(kDiffusionContextDim), 0.0F);
  encodeDiffusionObstacleContext(
    ego_x, ego_y, ego_yaw, ego_vel, boundary_left, boundary_right, obs_x.data(), obs_y.data(),
    obs_yaw.data(), obs_half_length.data(), obs_half_width.data(), nullptr, nullptr,
    static_cast<int>(obs_x.size()), out.data());
  return out;
}

inline void encodeDiffusionObstacleContextFromParkedCars(
  const float ego_x, const float ego_y, const float ego_yaw, const float ego_vel,
  const float boundary_left, const float boundary_right,
  const std::vector<mppi::cost::ParkedCarObstacle> & cars, float * out)
{
  std::vector<float> obs_x;
  std::vector<float> obs_y;
  std::vector<float> obs_yaw;
  std::vector<float> obs_half_length;
  std::vector<float> obs_half_width;
  detail::gatherParkedCarObstacles(cars, obs_x, obs_y, obs_yaw, obs_half_length, obs_half_width);
  encodeDiffusionObstacleContext(
    ego_x, ego_y, ego_yaw, ego_vel, boundary_left, boundary_right, obs_x.data(), obs_y.data(),
    obs_yaw.data(), obs_half_length.data(), obs_half_width.data(), nullptr, nullptr,
    static_cast<int>(obs_x.size()), out);
}

}  // namespace diffusion
}  // namespace sampling_distributions
}  // namespace mppi

#endif  // MPPI__SAMPLING_DISTRIBUTIONS__DIFFUSION__OBSTACLE_CONTEXT_HPP_
