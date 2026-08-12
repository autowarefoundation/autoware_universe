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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__COLLISION_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__COLLISION_HPP_

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <limits>
#include <optional>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
/**
 * @brief CollisionHit represents a collision between a path segment and an obstacle's shape.
 */
struct CollisionHit
{
  CollisionHit(size_t segment_index, double distance)
  : segment_index(segment_index), distance(distance)
  {
  }

  size_t segment_index;  //!< Index of the path segment [0, path.size() - 2].
  double distance;       //!< Distance of the collision point from the start of the path segment.
};

/**
 * @brief Find first collision between predicted path and static obstacles considering OBB collision
 * detection.
 *
 * @param target Predicted object to check for collisions.
 * @param mode_index Index of the mode to check for collisions.
 * @param obstacles Vector of predicted objects to check for collisions.
 * @param speed_threshold Speed threshold for collision detection.
 * @return std::optional<CollisionHit> Optional collision hit information.
 */
std::optional<CollisionHit> find_collision(
  const autoware_perception_msgs::msg::PredictedObject & target, const size_t mode_index,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & obstacles,
  double speed_threshold = std::numeric_limits<double>::infinity());
}  // namespace autoware::predicted_path_postprocessor::processor
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__COLLISION_HPP_
