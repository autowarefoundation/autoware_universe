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

#ifndef AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__AGENT_HPP_
#define AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__AGENT_HPP_

#include "autoware/simpl_prediction/archetype/exception.hpp"
#include "autoware/simpl_prediction/archetype/fixed_queue.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace autoware::simpl_prediction::archetype
{
/**
 * @brief An enumerate to represent agent labels.
 */
enum class AgentLabel {
  VEHICLE = 0,        //!< Normal size vehicle.
  PEDESTRIAN = 1,     //!< Pedestrian.
  MOTORCYCLIST = 2,   //!< Motorcyclist.
  CYCLIST = 3,        //!< Cyclist.
  LARGE_VEHICLE = 4,  //!< Large size vehicle such as bus.
  UNKNOWN = 5         //!< Catch all other agents.
};

/**
 * @brief Convert string label names to IDs.
 *
 * @param label_names Label names.
 */
std::vector<size_t> to_label_ids(const std::vector<std::string> & label_names);

/**
 * @brief A class to represent a state at the specific timestamp.
 */
struct AgentState
{
  /**
   * @brief Construct a new AgentState object with default values.
   */
  AgentState() = default;

  /**
   * @brief Construct a new AgentState object.
   *
   * @param x Location x.
   * @param y Location y.
   * @param z Location z.
   * @param yaw Yaw angle [rad].
   * @param vx X-direction velocity [m/s]
   * @param vy Y-direction velocity [m/s].
   * @param is_valid Indicates whther the state is valid.
   */
  AgentState(double x, double y, double z, double yaw, double vx, double vy, bool is_valid)
  : x(x), y(y), z(z), yaw(yaw), vx(vx), vy(vy), is_valid(is_valid)
  {
  }

  /**
   * @brief Return the number of state attributes, which is `8`.
   */
  static size_t num_attribute() { return 8; }

  /**
   * @brief Return the distance from another agent.
   *
   * @param other Another agent state.
   */
  double distance_from(const AgentState & other) const
  {
    return std::hypot(x - other.x, y - other.y);
  }

  /**
   * @brief Transform state to input state coordinate frame.
   *
   * @param to_state Agent state in the target coordinate frame.
   */
  AgentState transform(const AgentState & to_state) const;

  double x{0.0};         //!< Center x.
  double y{0.0};         //!< Center y.
  double z{0.0};         //!< Center z.
  double yaw{0.0};       //!< Yaw angle [rad].
  double vx{0.0};        //!< X-direction velocity [m/s].
  double vy{0.0};        //!< Y-direction velocity [m/s].
  bool is_valid{false};  //!< Indicates whether the state is valid.
};

/**
 * @brief Data container of state history for each agent.
 */
class AgentHistory
{
public:
  using value_type = AgentState;
  using size_type = FixedQueue<value_type>::size_type;
  using reference = FixedQueue<value_type>::reference;
  using const_reference = FixedQueue<value_type>::const_reference;
  using iterator = FixedQueue<value_type>::iterator;
  using const_iterator = FixedQueue<value_type>::const_iterator;

  /**
   * @brief Construct a new AgentHistory object with a single state.
   *
   * @param agent_id Agent ID.
   * @param label Agent label.
   * @param num_past Number of past timestamps.
   */
  AgentHistory(const std::string & agent_id, const AgentLabel & label, size_t num_past)
  : agent_id(agent_id), label(label), queue_(num_past)
  {
  }

  /**
   * @brief Construct a new AgentHistory object with a single state.
   *
   * @param agent_id Agent ID.
   * @param label Agent label.
   * @param num_past Number of past timestamps.
   * @param state Current agent state.
   */
  AgentHistory(
    const std::string & agent_id, const AgentLabel & label, size_t num_past,
    const value_type & state)
  : agent_id(agent_id), label(label), queue_(num_past)
  {
    update(state);
  }

  /**
   * @brief Update history with the latest state.
   *
   * @param state Current agent state.
   */
  void update(const value_type & state) noexcept { queue_.push_back(state); }

  /**
   * @brief Return distance from the current state to another agent.
   *
   * @param other Another agent state.
   */
  double distance_from(const AgentState & other) const { return current().distance_from(other); }

  /**
   * @brief Transform states to current coordinate frame.
   */
  AgentHistory transform_to_current() const;

  /**
   * @brief Return a read/write reference that points to the current state.
   */
  reference current() noexcept { return queue_.back(); }

  /**
   * @brief Return a read only reference that points to the current state.
   */
  const_reference current() const noexcept { return queue_.back(); }

  /**
   * @brief Return the number of past timestamps.
   */
  size_type size() const noexcept { return queue_.size(); }

  /**
   * @brief Return the read/write reference to the data at the specified time index.
   *
   * @param t Time index of the history.
   */
  reference at(size_type t) noexcept { return queue_.at(t); }

  /**
   * @brief Return the read only reference to the data at the specified time index.
   *
   * @param t Time index of the history.
   */
  const_reference at(size_type t) const noexcept { return queue_.at(t); }

  /**
   * @brief Return a read/write iterator that points the oldest state.
   */
  iterator begin() noexcept { return queue_.begin(); }

  /**
   * @brief Return a read only iterator that points the oldest state.
   */
  const_iterator begin() const noexcept { return queue_.begin(); }

  /**
   * @brief Return a read/write iterator that points the current state.
   */
  iterator end() noexcept { return queue_.end(); }

  /**
   * @brief Return a read only iterator that points the current state.
   */
  const_iterator end() const noexcept { return queue_.end(); }

  std::string agent_id;  //!< Agent ID.
  AgentLabel label;      //!< Agent label.

private:
  FixedQueue<value_type> queue_;  //!< Agent state container.
};

/**
 * @brief Trim top-k nearest neighbor agent histories by comparing the distance from the current
 * states.
 *
 * The pipeline is as follows:
 * 1. Filter histories by label IDs.
 * 2. Sort the histories by distance.
 * 3. Trim the top-k nearest neighbors.
 *
 * @param histories Source histories.
 * @param label_ids Label IDs of the agents to be considered.
 * @param state_from Agent state.
 * @param top_k Maximum number of agents to be trimmed.
 */
std::vector<AgentHistory> trim_neighbors(
  const std::vector<AgentHistory> & histories, const std::vector<size_t> & label_ids,
  const AgentState & state_from, size_t top_k);
}  // namespace autoware::simpl_prediction::archetype
#endif  // AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__AGENT_HPP_
