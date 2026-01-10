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

#include "autoware/diffusion_planner/conversion/agent.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::diffusion_planner
{

namespace
{

AgentLabel get_model_label(const TrackedObject & object)
{
  const uint8_t autoware_label =
    autoware::object_recognition_utils::getHighestProbLabel(object.classification);

  switch (autoware_label) {
    case autoware_perception_msgs::msg::ObjectClassification::CAR:
    case autoware_perception_msgs::msg::ObjectClassification::TRUCK:
    case autoware_perception_msgs::msg::ObjectClassification::BUS:
    case autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
    case autoware_perception_msgs::msg::ObjectClassification::TRAILER:
      return AgentLabel::VEHICLE;
    case autoware_perception_msgs::msg::ObjectClassification::BICYCLE:
      return AgentLabel::BICYCLE;
    case autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
      return AgentLabel::PEDESTRIAN;
    default:
      return AgentLabel::VEHICLE;
  }
}

bool is_unknown_object(const TrackedObject & object)
{
  const auto autoware_label =
    autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  return autoware_label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
}

}  // namespace

AgentState::AgentState(const TrackedObject & object, const rclcpp::Time & timestamp)
: timestamp(timestamp), original_info(object)
{
  position = object.kinematics.pose_with_covariance.pose.position;
  const float yaw =
    autoware_utils_geometry::get_rpy(object.kinematics.pose_with_covariance.pose.orientation).z;
  cos_yaw = std::cos(yaw);
  sin_yaw = std::sin(yaw);
  velocity = object.kinematics.twist_with_covariance.twist.linear;
  label = get_model_label(object);
  object_id = autoware_utils_uuid::to_hex_string(object.object_id);
}

void AgentState::apply_transform(const Eigen::Matrix4d & transform)
{
  Eigen::Vector4d pos_vec(position.x, position.y, position.z, 1.0);
  Eigen::Vector4d transformed_pos = transform * pos_vec;
  position.x = transformed_pos.x();
  position.y = transformed_pos.y();
  position.z = transformed_pos.z();

  Eigen::Vector4d dir_vec(cos_yaw, sin_yaw, 0.0, 0.0);
  Eigen::Vector4d transformed_dir = transform * dir_vec;
  cos_yaw = transformed_dir.x();
  sin_yaw = transformed_dir.y();

  const double velocity_norm = std::hypot(velocity.x, velocity.y);
  velocity.x = velocity_norm * cos_yaw;
  velocity.y = velocity_norm * sin_yaw;
}

// Return the state attribute as an array.
[[nodiscard]] std::array<float, AGENT_STATE_DIM> AgentState::as_array() const noexcept
{
  return {
    static_cast<float>(position.x),
    static_cast<float>(position.y),
    cos_yaw,
    sin_yaw,
    static_cast<float>(velocity.x),
    static_cast<float>(velocity.y),
    static_cast<float>(original_info.shape.dimensions.y),  // width
    static_cast<float>(original_info.shape.dimensions.x),  // length
    static_cast<float>(label == AgentLabel::VEHICLE),
    static_cast<float>(label == AgentLabel::PEDESTRIAN),
    static_cast<float>(label == AgentLabel::BICYCLE),
  };
}

void AgentData::update_histories(const TrackedObjects & objects, const bool ignore_unknown_agents)
{
  const rclcpp::Time objects_timestamp(objects.header.stamp);
  std::vector<std::string> found_ids;
  for (auto object : objects.objects) {
    if (ignore_unknown_agents && is_unknown_object(object)) {
      continue;
    }
    auto object_id = autoware_utils_uuid::to_hex_string(object.object_id);
    auto it = histories_map_.find(object_id);
    if (it != histories_map_.end()) {
      it->second.update(object, objects_timestamp);
    } else {
      histories_map_.emplace(object_id, AgentHistory(INPUT_T_WITH_CURRENT));
      histories_map_.at(object_id).fill(AgentState(object, objects_timestamp));
    }
    found_ids.push_back(object_id);
  }
  // Remove histories that are not found in the current objects
  for (auto it = histories_map_.begin(); it != histories_map_.end();) {
    if (std::find(found_ids.begin(), found_ids.end(), it->first) == found_ids.end()) {
      it = histories_map_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<AgentHistory> AgentData::transformed_and_trimmed_histories(
  const Eigen::Matrix4d & transform, size_t max_num_agent) const
{
  std::vector<AgentHistory> histories;
  histories.reserve(histories_map_.size());
  for (const auto & [_, history] : histories_map_) {
    histories.push_back(history);
  }
  for (auto & history : histories) {
    history.apply_transform(transform);
  }

  geometry_msgs::msg::Point position;
  position.x = 0.0;
  position.y = 0.0;
  position.z = 0.0;

  std::sort(
    histories.begin(), histories.end(),
    [&position](const AgentHistory & a, const AgentHistory & b) {
      return autoware_utils_geometry::calc_distance2d(position, a.get_latest_state().position) <
             autoware_utils_geometry::calc_distance2d(position, b.get_latest_state().position);
    });
  if (histories.size() > max_num_agent) {
    histories.erase(
      histories.begin() + static_cast<std::ptrdiff_t>(max_num_agent), histories.end());
  }
  return histories;
}

}  // namespace autoware::diffusion_planner
