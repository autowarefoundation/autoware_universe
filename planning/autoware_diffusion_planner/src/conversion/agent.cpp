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
#include "autoware/diffusion_planner/utils/utils.hpp"

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
: pose(utils::pose_to_matrix4d(object.kinematics.pose_with_covariance.pose)),
  timestamp(timestamp),
  label(get_model_label(object)),
  object_id(autoware_utils_uuid::to_hex_string(object.object_id)),
  original_info(object)
{
}

// Return the state attribute as an array.
[[nodiscard]] std::array<float, AGENT_STATE_DIM> AgentState::as_array() const noexcept
{
  const auto [cos_yaw, sin_yaw] = utils::rotation_matrix_to_cos_sin(pose.block<3, 3>(0, 0));
  const auto & linear_vel = original_info.kinematics.twist_with_covariance.twist.linear;
  const double velocity_norm = std::hypot(linear_vel.x, linear_vel.y);
  const double velocity_x = velocity_norm * cos_yaw;
  const double velocity_y = velocity_norm * sin_yaw;

  return {
    static_cast<float>(pose(0, 3)),
    static_cast<float>(pose(1, 3)),
    static_cast<float>(cos_yaw),
    static_cast<float>(sin_yaw),
    static_cast<float>(velocity_x),
    static_cast<float>(velocity_y),
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
  for (const TrackedObject & object : objects.objects) {
    if (ignore_unknown_agents && is_unknown_object(object)) {
      continue;
    }
    const std::string object_id = autoware_utils_uuid::to_hex_string(object.object_id);
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

  std::sort(histories.begin(), histories.end(), [](const AgentHistory & a, const AgentHistory & b) {
    const double a_dist =
      std::hypot(a.get_latest_state().pose(0, 3), a.get_latest_state().pose(1, 3));
    const double b_dist =
      std::hypot(b.get_latest_state().pose(0, 3), b.get_latest_state().pose(1, 3));
    return a_dist < b_dist;
  });
  if (histories.size() > max_num_agent) {
    histories.erase(
      histories.begin() + static_cast<std::ptrdiff_t>(max_num_agent), histories.end());
  }
  return histories;
}

}  // namespace autoware::diffusion_planner
