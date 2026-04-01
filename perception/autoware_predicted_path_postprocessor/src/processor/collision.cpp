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

#include "autoware/predicted_path_postprocessor/processor/collision.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometric_shapes/obb.h>

#include <cassert>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
namespace
{
/**
 * @brief Convert a pose and extents to an OBB.
 *
 * @param pose Pose of the object.
 * @param extents Extents of the object.
 * @return bodies::OBB OBB representation of the object.
 */
bodies::OBB to_obb(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & extents)
{
  Eigen::Isometry3d pose_eigen;
  tf2::fromMsg(pose, pose_eigen);
  Eigen::Vector3d extents_eigen(extents.x, extents.y, extents.z);
  return bodies::OBB(pose_eigen, extents_eigen);
}

/**
 * @brief Convert a path and extents to an OBB.
 *
 * @param path Path of the object.
 * @param index Index of the pose to use for the OBB.
 * @param path_extents Extents of the object.
 * @return bodies::OBB OBB representation of the object.
 */
bodies::OBB to_obb(
  const std::vector<geometry_msgs::msg::Pose> & path, const size_t index,
  const geometry_msgs::msg::Vector3 & path_extents)
{
  auto yaw_from = [&path](size_t i) {
    assert(i + 1 < path.size() && "yaw_from requires valid next index");
    const auto & p1 = path[i];
    const auto & p2 = path[i + 1];
    const auto dx = p2.position.x - p1.position.x;
    const auto dy = p2.position.y - p1.position.y;
    return std::atan2(dy, dx);
  };

  auto pose = path[index];
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw_from(index));

  return to_obb(pose, path_extents);
}
}  // namespace

std::optional<CollisionHit> find_collision(
  const autoware_perception_msgs::msg::PredictedObject & target, const size_t mode_index,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & obstacles,
  double speed_threshold)
{
  constexpr double epsilon = 1e-6;
  double min_collision_distance = std::numeric_limits<double>::infinity();
  double global_distance = 0.0;
  std::optional<CollisionHit> result = std::nullopt;
  const auto & mode = target.kinematics.predicted_paths[mode_index];
  for (size_t i = 0; i + 1 < mode.path.size(); ++i) {
    const auto & current = mode.path[i].position;
    const auto & next = mode.path[i + 1].position;

    const auto segment_length = std::hypot(next.x - current.x, next.y - current.y);
    if (segment_length < epsilon) {
      global_distance += segment_length;
      continue;
    }

    const auto target_obb = to_obb(mode.path, i, target.shape.dimensions);

    for (const auto & obstacle : obstacles) {
      // if the obstacle has a high speed or is the same as the path, skip it
      if (auto speed = std::abs(obstacle.kinematics.initial_twist_with_covariance.twist.linear.x);
          speed > speed_threshold || obstacle.object_id == target.object_id) {
        continue;
      }

      const auto obstacle_obb =
        to_obb(obstacle.kinematics.initial_pose_with_covariance.pose, obstacle.shape.dimensions);

      // check segment intersection: assumes the current global distance is the collision distance
      if (target_obb.overlaps(obstacle_obb)) {
        if (global_distance + epsilon < min_collision_distance) {
          min_collision_distance = global_distance;
          result.emplace(i, global_distance);
        }
      }
    }
    // accumulate global distance between current and next points
    global_distance += segment_length;
  }
  return result;
}
}  // namespace autoware::predicted_path_postprocessor::processor
