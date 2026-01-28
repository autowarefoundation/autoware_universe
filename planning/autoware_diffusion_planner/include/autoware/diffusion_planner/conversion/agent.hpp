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

#ifndef AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HPP_

#include "Eigen/Dense"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/detail/tracked_objects__struct.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
namespace autoware::diffusion_planner
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
constexpr size_t AGENT_STATE_DIM = 11;

enum AgentLabel { VEHICLE = 0, PEDESTRIAN = 1, BICYCLE = 2 };

/**
 * @brief A class to represent a single state of an agent.
 */
struct AgentState
{
  AgentState() = default;

  AgentState(const TrackedObject & object, const rclcpp::Time & timestamp);

  [[nodiscard]] std::array<float, AGENT_STATE_DIM> as_array() const noexcept;

  // Only the pose is mutable (by `apply_transform` in AgentHistory)
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};

  const rclcpp::Time timestamp;
  const AgentLabel label{AgentLabel::VEHICLE};
  const std::string object_id;
  const TrackedObject original_info;
};

/**
 * @brief A class to represent the state history of an agent.
 */
struct AgentHistory
{
  explicit AgentHistory(const size_t max_size) : max_size_(max_size) {}

  void fill(const AgentState & state)
  {
    while (!full()) {
      push_back(state);
    }
  }

  void update(const TrackedObject & object, const rclcpp::Time & timestamp)
  {
    AgentState state(object, timestamp);
    if (
      queue_.size() > 0 &&
      queue_.back().object_id != autoware_utils_uuid::to_hex_string(object.object_id)) {
      throw std::runtime_error("Object ID mismatch");
    }
    push_back(state);
  }

  [[nodiscard]] std::vector<float> as_array() const noexcept
  {
    std::vector<float> output;
    for (const auto & state : queue_) {
      for (const auto & v : state.as_array()) {
        output.push_back(v);
      }
    }
    return output;
  }

  [[nodiscard]] const AgentState & get_latest_state() const { return queue_.back(); }

  void apply_transform(const Eigen::Matrix4d & transform)
  {
    for (auto & state : queue_) {
      state.pose = transform * state.pose;
    }
  }

private:
  void push_back(const AgentState & state)
  {
    if (full()) {
      queue_.pop_front();
    }
    queue_.push_back(state);
  }

  bool full() const { return queue_.size() >= max_size_; }

  std::deque<AgentState> queue_;
  size_t max_size_{0};
};

/**
 * @brief A class containing whole state histories of all agent.
 */
struct AgentData
{
  void update_histories(const TrackedObjects & objects, const bool ignore_unknown_agents);

  // Transform histories, trim to max_num_agent, and return the processed vector.
  std::vector<AgentHistory> transformed_and_trimmed_histories(
    const Eigen::Matrix4d & transform, size_t max_num_agent) const;

private:
  std::unordered_map<std::string, AgentHistory> histories_map_;
};

// Convert histories to a flattened vector
inline std::vector<float> flatten_histories_to_vector(
  const std::vector<AgentHistory> & histories, size_t max_num_agent, size_t time_length)
{
  std::vector<float> data;
  data.reserve(histories.size() * time_length * AGENT_STATE_DIM);

  for (const auto & history : histories) {
    const auto history_array = history.as_array();
    data.insert(data.end(), history_array.begin(), history_array.end());
  }

  data.resize(max_num_agent * time_length * AGENT_STATE_DIM, 0.0f);

  return data;
}

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HPP_
