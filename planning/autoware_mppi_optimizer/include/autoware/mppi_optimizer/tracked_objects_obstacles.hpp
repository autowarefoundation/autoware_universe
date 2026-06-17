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

#ifndef AUTOWARE__MPPI_OPTIMIZER__TRACKED_OBJECTS_OBSTACLES_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__TRACKED_OBJECTS_OBSTACLES_HPP_

#include "autoware/mppi_optimizer/predicted_objects_obstacles.hpp"

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace autoware::mppi_optimizer
{

using autoware_perception_msgs::msg::TrackedObjects;

/**
 * @brief Extrapolate perception tracked objects with constant longitudinal velocity.
 *
 * Twist is treated as object-frame longitudinal speed (linear.x), matching common Autoware usage.
 * Up to kMaxMppiObstacles objects are used.
 */
inline void buildObstacleTrajectoryBuffersFromTrackedObjects(
  const TrackedObjects & objects, const float dt, const int num_timesteps, std::vector<float> & x,
  std::vector<float> & y, std::vector<float> & yaw, std::vector<float> & half_length,
  std::vector<float> & half_width)
{
  const size_t obstacle_count =
    std::min(objects.objects.size(), static_cast<size_t>(kMaxMppiObstacles));
  const int nt = std::max(1, num_timesteps);

  x.assign(obstacle_count * static_cast<size_t>(nt), -1.0E4F);
  y.assign(obstacle_count * static_cast<size_t>(nt), -1.0E4F);
  yaw.assign(obstacle_count * static_cast<size_t>(nt), 0.0F);
  half_length.assign(obstacle_count, 0.0F);
  half_width.assign(obstacle_count, 0.0F);

  for (size_t obstacle_idx = 0; obstacle_idx < obstacle_count; ++obstacle_idx) {
    const auto & object = objects.objects[obstacle_idx];
    const auto & pose = object.kinematics.pose_with_covariance.pose;
    const float x0 = static_cast<float>(pose.position.x);
    const float y0 = static_cast<float>(pose.position.y);
    const float object_yaw = static_cast<float>(tf2::getYaw(pose.orientation));
    const float speed = static_cast<float>(object.kinematics.twist_with_covariance.twist.linear.x);

    float length = kDefaultObstacleLength;
    float width = kDefaultObstacleWidth;
    obstacleFootprintFromShape(object.shape, length, width);
    half_length[obstacle_idx] = length * 0.5F;
    half_width[obstacle_idx] = width * 0.5F;

    const float vx = speed * std::cos(object_yaw);
    const float vy = speed * std::sin(object_yaw);

    for (int timestep = 0; timestep < nt; ++timestep) {
      const size_t buffer_idx =
        obstacle_idx * static_cast<size_t>(nt) + static_cast<size_t>(timestep);
      const float relative_time = static_cast<float>(timestep) * dt;
      x[buffer_idx] = x0 + vx * relative_time;
      y[buffer_idx] = y0 + vy * relative_time;
      yaw[buffer_idx] = object_yaw;
    }
  }
}

inline int trackedObjectObstacleCount(const TrackedObjects & objects)
{
  return static_cast<int>(std::min(objects.objects.size(), static_cast<size_t>(kMaxMppiObstacles)));
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__TRACKED_OBJECTS_OBSTACLES_HPP_
