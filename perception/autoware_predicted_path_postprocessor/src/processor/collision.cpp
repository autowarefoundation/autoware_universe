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

#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
std::optional<CollisionHit> find_collision(
  const autoware_perception_msgs::msg::PredictedPath & path,
  const unique_identifier_msgs::msg::UUID & path_uuid,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & obstacles,
  double speed_threshold)
{
  constexpr double epsilon = 1e-6;
  double min_collision_distance = std::numeric_limits<double>::infinity();
  double global_distance = 0.0;
  std::optional<CollisionHit> result = std::nullopt;
  for (size_t i = 0; i < path.path.size() - 1; ++i) {
    const auto & current = path.path[i].position;
    const auto & next = path.path[i + 1].position;

    const auto segment_length = std::hypot(next.x - current.x, next.y - current.y);
    if (segment_length < epsilon) {
      continue;
    }

    for (const auto & obstacle : obstacles) {
      // if the obstacle has a high speed or is the same as the path, skip it
      if (auto speed = abs(obstacle.kinematics.initial_twist_with_covariance.twist.linear.x);
          speed > speed_threshold || obstacle.object_id == path_uuid) {
        continue;
      }

      // check segment intersection
      const auto polygon = autoware_utils_geometry::to_polygon2d(obstacle).outer();
      for (size_t j = 0; j < polygon.size() - 1; ++j) {
        const auto & point1 =
          autoware_utils_geometry::create_point(polygon[j].x(), polygon[j].y(), 0.0);
        const auto & point2 =
          autoware_utils_geometry::create_point(polygon[j + 1].x(), polygon[j + 1].y(), 0.0);

        const auto intersection = autoware_utils_geometry::intersect(current, next, point1, point2);

        if (!intersection) {
          continue;
        }

        // local segment distance + global distance
        const auto collision_distance =
          std::hypot(intersection->x - current.x, intersection->y - current.y) + global_distance;

        // update result if the collision distance is less than the minimum collision distance
        if (collision_distance + epsilon < min_collision_distance) {
          min_collision_distance = collision_distance;
          result = CollisionHit{i, collision_distance};
        }
      }
    }

    // accumulate global distance between current and next points
    global_distance += segment_length;
  }
  return result;
}
}  // namespace autoware::predicted_path_postprocessor::processor
