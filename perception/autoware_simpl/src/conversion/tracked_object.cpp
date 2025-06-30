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

#include "autoware/simpl/conversion/tracked_object.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <cmath>
#include <utility>

namespace autoware::simpl::conversion
{
namespace
{
/**
 * @brief Return `archetype::AgentLabel` from `TrackedObject` message.
 *
 * @param object Tracked object.
 */
archetype::AgentLabel to_agent_label(const autoware_perception_msgs::msg::TrackedObject & object)
{
  using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;

  const auto label = object_recognition_utils::getHighestProbLabel(object.classification);
  if (
    object_recognition_utils::isCarLikeVehicle(label) &&
    !object_recognition_utils::isLargeVehicle(label)) {
    return archetype::AgentLabel::VEHICLE;
  } else if (object_recognition_utils::isLargeVehicle(label)) {
    return archetype::AgentLabel::LARGE_VEHICLE;
  } else if (label == ObjectClassification::PEDESTRIAN) {
    return archetype::AgentLabel::PEDESTRIAN;
  } else if (label == ObjectClassification::MOTORCYCLE) {
    return archetype::AgentLabel::MOTORCYCLIST;
  } else if (label == ObjectClassification::BICYCLE) {
    return archetype::AgentLabel::CYCLIST;
  } else {
    return archetype::AgentLabel::UNKNOWN;
  }
}

/**
 * @brief Transform velocity coordinate frame from local to global coordinate frame.
 *
 * @param velocity Linear velocity vector.
 * @param yaw Yaw angle [rad] in global coordinate frame.
 */
std::pair<double, double> local_to_global(const geometry_msgs::msg::Vector3 & velocity, double yaw)
{
  const auto vcos = std::cos(yaw);
  const auto vsin = std::sin(yaw);

  const auto vx = velocity.x * vcos - velocity.y * vsin;
  const auto vy = velocity.x * vsin + velocity.y * vcos;

  return {vx, vy};
}
}  // namespace

archetype::AgentState to_agent_state(const autoware_perception_msgs::msg::TrackedObject & object)
{
  const auto & pose = object.kinematics.pose_with_covariance.pose;
  const double yaw = autoware_utils_geometry::get_rpy(pose.orientation).z;
  const auto [vx, vy] = local_to_global(object.kinematics.twist_with_covariance.twist.linear, yaw);
  const auto label = to_agent_label(object);

  return {pose.position.x, pose.position.y, pose.position.z, yaw, vx, vy, label, true};
}

archetype::AgentState to_agent_state(const nav_msgs::msg::Odometry & odometry)
{
  const auto & pose = odometry.pose.pose;
  const double yaw = autoware_utils_geometry::get_rpy(pose.orientation).z;
  const auto [vx, vy] = local_to_global(odometry.twist.twist.linear, yaw);

  return {
    pose.position.x,
    pose.position.y,
    pose.position.z,
    yaw,
    vx,
    vy,
    archetype::AgentLabel::VEHICLE,
    true};
}
}  // namespace autoware::simpl::conversion
