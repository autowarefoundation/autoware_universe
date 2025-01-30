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

#include "autoware/simpl/archetype/agent.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <tf2/utils.h>

namespace autoware::simpl::conversion
{
namespace
{
archetype::AgentLabel to_label(const autoware_perception_msgs::msg::TrackedObject & object)
{
  using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;

  const auto label = object_recognition_utils::getHighestProbLabel(object.classification);
  if (
    label == object_recognition_utils::isCarLikeVehicle(label) &&
    label != object_recognition_utils::isLargeVehicle(label)) {
    return archetype::AgentLabel::VEHICLE;
  } else if (label == object_recognition_utils::isLargeVehicle(label)) {
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
}  // namespace

archetype::AgentState to_state(
  const autoware_perception_msgs::msg::TrackedObject & object, bool is_valid)
{
  const auto & pose = object.kinematics.pose_with_covariance.pose;
  const auto yaw = tf2::getYaw(pose.orientation);
  const auto & velocity = object.kinematics.twist_with_covariance.twist.linear;
  const auto label = to_label(object);

  return {pose.position, yaw, velocity, label, is_valid};
}
}  // namespace autoware::simpl::conversion
