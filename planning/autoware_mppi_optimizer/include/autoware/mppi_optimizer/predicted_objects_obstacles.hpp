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

#ifndef AUTOWARE__MPPI_OPTIMIZER__PREDICTED_OBJECTS_OBSTACLES_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__PREDICTED_OBJECTS_OBSTACLES_HPP_

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace autoware::mppi_optimizer
{

using autoware_perception_msgs::msg::PredictedObjects;

constexpr size_t kMaxMppiObstacles = 64U;
constexpr float kDefaultObstacleLength = 0.55F * 1.5F;
constexpr float kDefaultObstacleWidth = 0.28F * 1.5F;

inline float durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<float>(duration.sec) + static_cast<float>(duration.nanosec) * 1.0e-9F;
}

inline const autoware_perception_msgs::msg::PredictedPath * selectPredictedPath(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  if (object.kinematics.predicted_paths.empty()) {
    return nullptr;
  }
  const auto best_it = std::max_element(
    object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  return &(*best_it);
}

inline void obstacleFootprintFromShape(
  const autoware_perception_msgs::msg::Shape & shape, float & length, float & width)
{
  length =
    shape.dimensions.x > 1.0e-3 ? static_cast<float>(shape.dimensions.x) : kDefaultObstacleLength;
  width =
    shape.dimensions.y > 1.0e-3 ? static_cast<float>(shape.dimensions.y) : kDefaultObstacleWidth;
}

/**
 * @brief Sample diffusion-predicted object paths into MPPI obstacle-major trajectory buffers.
 *
 * Predicted paths are interpreted relative to the current planning cycle (t = 0 at inference).
 * Up to kMaxMppiObstacles objects are used.
 */
inline void buildObstacleTrajectoryBuffersFromPredictedObjects(
  const PredictedObjects & objects, const float dt, const int num_timesteps, std::vector<float> & x,
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
    float length = kDefaultObstacleLength;
    float width = kDefaultObstacleWidth;
    obstacleFootprintFromShape(object.shape, length, width);
    half_length[obstacle_idx] = length * 0.5F;
    half_width[obstacle_idx] = width * 0.5F;

    const auto * predicted_path = selectPredictedPath(object);
    const auto & fallback_pose = object.kinematics.initial_pose_with_covariance.pose;
    const float fallback_yaw = static_cast<float>(tf2::getYaw(fallback_pose.orientation));

    for (int timestep = 0; timestep < nt; ++timestep) {
      const size_t buffer_idx =
        obstacle_idx * static_cast<size_t>(nt) + static_cast<size_t>(timestep);
      const float relative_time = static_cast<float>(timestep) * dt;

      if (predicted_path != nullptr && !predicted_path->path.empty()) {
        const float path_dt = std::max(durationSeconds(predicted_path->time_step), 1.0e-3F);
        const int path_idx = std::min(
          static_cast<int>(std::lround(relative_time / path_dt)),
          static_cast<int>(predicted_path->path.size()) - 1);
        const auto & pose = predicted_path->path[static_cast<size_t>(path_idx)];
        x[buffer_idx] = static_cast<float>(pose.position.x);
        y[buffer_idx] = static_cast<float>(pose.position.y);
        yaw[buffer_idx] = static_cast<float>(tf2::getYaw(pose.orientation));
      } else {
        x[buffer_idx] = static_cast<float>(fallback_pose.position.x);
        y[buffer_idx] = static_cast<float>(fallback_pose.position.y);
        yaw[buffer_idx] = fallback_yaw;
      }
    }
  }
}

inline int predictedObjectObstacleCount(const PredictedObjects & objects)
{
  return static_cast<int>(std::min(objects.objects.size(), static_cast<size_t>(kMaxMppiObstacles)));
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__PREDICTED_OBJECTS_OBSTACLES_HPP_
