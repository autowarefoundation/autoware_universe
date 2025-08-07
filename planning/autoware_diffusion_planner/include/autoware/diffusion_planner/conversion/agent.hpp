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
#include "autoware/diffusion_planner/utils/fixed_queue.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/detail/tracked_objects__struct.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
using autoware_perception_msgs::msg::TrackedObject;

using autoware_perception_msgs::msg::ObjectClassification;
constexpr size_t AGENT_STATE_DIM = 11;

enum AgentLabel { VEHICLE = 0, PEDESTRIAN = 1, BICYCLE = 2 };

// enum AgentDimLabels {
//   X = 0,
//   Y = 1,
//   COS_YAW = 2,
//   SIN_YAW = 3,
//   VX = 4,
//   VY = 5,
//   L = 6,
//   W = 7,
//   LABEL_VEHICLE = 8,
//   LABEL_PEDESTRIAN = 9,
//   LABEL_BICYCLE = 10,
// };

AgentLabel get_model_label(const autoware_perception_msgs::msg::TrackedObject & object);

/**
 * @brief A class to represent a single state of an agent.
 */
struct AgentState
{
  // Construct a new instance filling all elements by `0.0f`.
  AgentState() = default;

  explicit AgentState(TrackedObject & object);

  /**
   * @brief Construct a new instance with specified values.
   *
   * @param position 3D position [m].
   * @param dimension Box dimension [m].
   * @param yaw Heading yaw angle [rad].
   * @param velocity Velocity [m/s].
   * @param label Agent label
   */
  // AgentState(
  //   const geometry_msgs::msg::Point & position, const geometry_msgs::msg::Vector3 & dimension,
  //   float yaw, const geometry_msgs::msg::Vector3 & velocity, const AgentLabel & label,
  //   std::string object_id);

  // Construct a new instance filling all elements by `0.0f`.
  static AgentState empty() noexcept { return {}; }

  // Return the agent state dimensions `D`.
  static size_t dim() { return AGENT_STATE_DIM; }

  // Return shape
  [[nodiscard]] autoware_perception_msgs::msg::Shape shape() const { return shape_; }

  // Return the x position.
  [[nodiscard]] float x() const { return static_cast<float>(position_.x); }

  // Return the y position.
  [[nodiscard]] float y() const { return static_cast<float>(position_.y); }

  // Return the z position.
  [[nodiscard]] float z() const { return static_cast<float>(position_.z); }

  // Return the length of object size.
  [[nodiscard]] float length() const { return static_cast<float>(shape_.dimensions.x); }

  // Return the width of object size.
  [[nodiscard]] float width() const { return static_cast<float>(shape_.dimensions.y); }

  // Return the cos of yaw.
  [[nodiscard]] float cos_yaw() const { return cos_yaw_; }

  // Return the sin of yaw.
  [[nodiscard]] float sin_yaw() const { return sin_yaw_; }

  // Return the x velocity.
  [[nodiscard]] float vx() const { return static_cast<float>(velocity_.x); }

  // Return the y velocity.
  [[nodiscard]] float vy() const { return static_cast<float>(velocity_.y); }

  // Return TrackedObject info
  [[nodiscard]] TrackedObject tracked_object() const { return tracked_object_info_; }

  void apply_transform(const Eigen::Matrix4f & transform);

  [[nodiscard]] std::string to_string() const;

  // Return the state attribute as an array.
  [[nodiscard]] std::array<float, AGENT_STATE_DIM> as_array() const noexcept;

  geometry_msgs::msg::Point position_;
  autoware_perception_msgs::msg::Shape shape_;
  float yaw_{0.0f};
  float cos_yaw_{0.0f};
  float sin_yaw_{0.0f};
  geometry_msgs::msg::Vector3 velocity_;
  AgentLabel label_{AgentLabel::VEHICLE};
  uint8_t autoware_label_;
  std::string object_id_;
  TrackedObject tracked_object_info_;
};

/**
 * @brief A class to represent the state history of an agent.
 */
struct AgentHistory
{
  /**
   * @brief Construct a new Agent History filling the latest state by input state.
   *
   * @param state Object current state.
   * @param object_id Object ID.
   * @param label_id Label ID.
   * @param current_time Current timestamp.
   * @param max_time_length History length.
   */
  AgentHistory(
    const AgentState & state, const size_t label_id, const double current_time,
    const size_t max_time_length, bool is_pad_history = true);

  // Return the history time length `T`.
  [[nodiscard]] size_t length() const { return max_size_; }

  // Return the number of agent state dimensions `D`.
  static size_t state_dim() { return AGENT_STATE_DIM; }

  // Return the data size of history `T * D`.
  [[nodiscard]] size_t size() const { return max_size_ * state_dim(); }

  // Return the shape of history matrix ordering in `(T, D)`.
  [[nodiscard]] std::tuple<size_t, size_t> shape() const { return {max_size_, state_dim()}; }

  // Return the object id.
  [[nodiscard]] const std::string & object_id() const { return object_id_; }

  // Return the label id.
  [[nodiscard]] size_t label_id() const { return label_id_; }

  // Return autoware_label
  [[nodiscard]] uint8_t autoware_label() const { return autoware_label_; }

  /**
   * @brief Return the last timestamp when non-empty state was pushed.
   *
   * @return double
   */
  [[nodiscard]] double latest_time() const { return latest_time_; }

  /**
   * @brief Update history with input state and latest time.
   *
   * @param current_time The current timestamp.
   * @param object The object info.
   */
  void update(double current_time, TrackedObject & object);
  /**
   * @brief Update history with input state and latest time.
   *
   * @param current_time The current timestamp.
   * @param state The current agent state.
   */
  void update(double current_time, const AgentState & state);

  // Update history with all-zeros state, but latest time is not updated.
  void update_empty() noexcept
  {
    const auto state = AgentState::empty();
    queue_.push_back(state);
  }

  // Return a history states as an array.
  [[nodiscard]] std::vector<float> as_array() const noexcept;

  /**
   * @brief Check whether the latest valid state is too old or not.
   *
   * @param current_time Current timestamp.
   * @param threshold Time difference threshold value.
   * @return true If the difference is greater than threshold.
   * @return false Otherwise
   */
  [[nodiscard]] bool is_ancient(double current_time, double threshold) const
  {
    /* TODO: Raise error if the current time is smaller than latest */
    return current_time - latest_time_ >= threshold;
  }

  // Get the latest agent state at `T`.
  [[nodiscard]] const AgentState & get_latest_state() const { return queue_.back(); }

  [[nodiscard]] const geometry_msgs::msg::Point & get_latest_state_position() const
  {
    return get_latest_state().position_;
  }

  [[nodiscard]] bool is_full() const { return queue_.size() >= max_size_; }

  void pad_history(bool pad_front = true);

  void apply_transform(const Eigen::Matrix4f & transform)
  {
    for (auto & state : queue_) {
      state.apply_transform(transform);
    }
  }

  [[nodiscard]] std::string to_string() const;

  // private:
  FixedQueue<AgentState> queue_;
  std::string object_id_;
  size_t label_id_;
  uint8_t autoware_label_;
  double latest_time_;
  size_t max_size_;
};

/**
 * @brief A class containing whole state histories of all agent.
 */
struct AgentData
{
  /**
   * @brief Construct a new instance.
   *
   * @param histories An array of histories for each object.
   * @param num_agent Number of agents.
   * @param num_timestamps Number of timestamps.
   */
  explicit AgentData(
    const autoware_perception_msgs::msg::TrackedObjects & objects, const size_t max_num_agent = 32,
    const size_t num_timestamps = 21, const bool ignore_unknown_agents = false);

  void apply_transform(const Eigen::Matrix4f & transform)
  {
    for (auto & history : histories_) {
      history.apply_transform(transform);
    }
    fill_data(histories_);
  }

  // fill data array
  void fill_data(const std::vector<AgentHistory> & histories, bool pad_with_zeroes = true);

  void update_histories(
    const autoware_perception_msgs::msg::TrackedObjects & objects,
    const bool ignore_unknown_agents = false);

  static bool is_unknown_object(const autoware_perception_msgs::msg::TrackedObject & object);

  void trim_to_k_closest_agents();
  void trim_to_k_closest_agents(const geometry_msgs::msg::Point & position);
  // Return the number of classes `C`.
  static size_t num_class() { return 3; }

  // Return the number of agents `N`.
  size_t num_agent() const { return num_agent_; }

  // Return the timestamp length `T`.
  size_t time_length() const { return time_length_; }

  // Return the number of agent state dimensions `D`.
  static size_t state_dim() { return AGENT_STATE_DIM; }

  // Return the number of all elements `N*T*D`.
  size_t size() const { return num_agent_ * time_length_ * state_dim(); }

  // Return the data shape ordering in (N, T, D).
  std::tuple<size_t, size_t, size_t> shape() const
  {
    return {num_agent_, time_length_, state_dim()};
  }

  [[nodiscard]] std::string to_string() const;

  // Return the address pointer of data array.
  const float * data_ptr() const noexcept { return data_.data(); }

  // Copy the data to a vector
  std::vector<float> as_vector() const noexcept { return data_; }

  std::vector<AgentHistory> get_histories() const { return histories_; }

private:
  std::vector<AgentHistory> histories_;
  std::unordered_map<std::string, size_t> histories_idx_map_;
  size_t num_agent_{0};
  size_t max_num_agent_{0};
  size_t time_length_{0};
  std::vector<float> data_;
};

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HPP_
