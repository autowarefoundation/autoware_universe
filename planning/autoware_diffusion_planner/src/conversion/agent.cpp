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

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/conversion/agent_history_resampler.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
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
      return AgentLabel::IGNORE;
  }
}

// Transform every history to the target frame, sort by distance to the frame origin (nearest
// first), and trim to at most max_num_agent entries.
std::vector<AgentHistory> transform_sort_trim(
  std::vector<AgentHistory> histories, const Eigen::Matrix4d & transform,
  const size_t max_num_agent)
{
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

void AgentData::update_histories(const TrackedObjects & objects)
{
  const rclcpp::Time objects_timestamp(objects.header.stamp);
  std::vector<std::string> found_ids;
  for (const TrackedObject & object : objects.objects) {
    if (get_model_label(object) == AgentLabel::IGNORE) {
      continue;
    }
    if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
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
  return transform_sort_trim(std::move(histories), transform, max_num_agent);
}

void AgentData::update_histories(
  const TrackedObjects & objects, [[maybe_unused]] const HistoryResamplingParams & params)
{
  const rclcpp::Time objects_timestamp(objects.header.stamp);

  // Beyond this age no retained observation can contribute a grid slot.
  constexpr double retention_horizon_s =
    static_cast<double>(INPUT_T_WITH_CURRENT - 1) * constants::PREDICTION_TIME_STEP_S;

  // A stamp regression beyond the retention horizon is a time reset (bag loop / sim restart):
  // drop all buffered state and ingest the message as the first one.
  if (
    last_processed_stamp_.has_value() &&
    (last_processed_stamp_.value() - objects_timestamp).seconds() > retention_horizon_s) {
    histories_map_.clear();
    latest_ids_.clear();
    last_processed_stamp_.reset();
  }

  // Dedup: the neighbor subscriber polls the latest message, so the same (stale) message can be
  // served on consecutive ticks. Only ingest messages whose stamp strictly advances.
  if (last_processed_stamp_.has_value() && objects_timestamp <= last_processed_stamp_.value()) {
    return;
  }
  last_processed_stamp_ = objects_timestamp;

  latest_ids_.clear();
  for (const TrackedObject & object : objects.objects) {
    if (get_model_label(object) == AgentLabel::IGNORE) {
      continue;
    }
    if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
      continue;
    }
    const std::string object_id = autoware_utils_uuid::to_hex_string(object.object_id);
    auto it = histories_map_.find(object_id);
    if (it != histories_map_.end()) {
      it->second.update(object, objects_timestamp);
    } else {
      // New UUID: start a growing buffer with a single real observation. No repeat-fill; the
      // resampling step extrapolates backward from this observation to fill the pre-appearance
      // grid.
      auto [inserted, _] =
        histories_map_.emplace(object_id, AgentHistory(NEIGHBOR_HISTORY_BUFFER_SIZE));
      inserted->second.update(object, objects_timestamp);
    }
    latest_ids_.insert(object_id);
  }

  // Prune histories whose newest observation has aged out of the history window; absent agents
  // within the window stay buffered so a re-published track resumes its real history.
  for (auto it = histories_map_.begin(); it != histories_map_.end();) {
    const double age_s = (objects_timestamp - it->second.get_latest_state().timestamp).seconds();
    if (age_s > retention_horizon_s) {
      it = histories_map_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<AgentHistory> AgentData::resampled_transformed_and_trimmed_histories(
  const rclcpp::Time & frame_time, const Eigen::Matrix4d & transform, size_t max_num_agent,
  const HistoryResamplingParams & params) const
{
  std::vector<AgentHistory> histories;
  histories.reserve(histories_map_.size());
  for (const auto & [object_id, history] : histories_map_) {
    // Emission mirrors the tracker's publication verdict: agents absent from the latest message
    // stay buffered but are withheld from the model.
    if (latest_ids_.count(object_id) == 0) {
      continue;
    }
    auto resampled = resample_history(history, frame_time, params);
    if (resampled.has_value()) {
      histories.push_back(std::move(resampled.value()));
    }
  }
  return transform_sort_trim(std::move(histories), transform, max_num_agent);
}

}  // namespace autoware::diffusion_planner
