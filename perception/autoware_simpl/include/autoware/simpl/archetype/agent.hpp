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

#ifndef AUTOWARE__SIMPL__ARCHETYPE__AGENT_HPP_
#define AUTOWARE__SIMPL__ARCHETYPE__AGENT_HPP_

#include "autoware/simpl/archetype/exception.hpp"
#include "autoware/simpl/archetype/fixed_queue.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <cmath>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::simpl::archetype
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

inline std::vector<size_t> to_label_ids(const std::vector<std::string> & label_names)
{
  std::vector<size_t> output;
  for (const auto & name : label_names) {
    if (name == "VEHICLE") {
      output.push_back(static_cast<size_t>(AgentLabel::VEHICLE));
    } else if (name == "PEDESTRIAN") {
      output.push_back(static_cast<size_t>(AgentLabel::PEDESTRIAN));
    } else if (name == "MOTORCYCLIST") {
      output.push_back(static_cast<size_t>(AgentLabel::MOTORCYCLIST));
    } else if (name == "CYCLIST") {
      output.push_back(static_cast<size_t>(AgentLabel::CYCLIST));
    } else if (name == "LARGE_VEHICLE") {
      output.push_back(static_cast<size_t>(AgentLabel::LARGE_VEHICLE));
    } else {
      output.push_back(static_cast<size_t>(AgentLabel::UNKNOWN));
    }
  }
  return output;
}

/**
 * @brief A class to represent a state at the specific timestamp.
 */
class AgentState
{
public:
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
   * @param label Label.
   * @param is_valid Indicates whther the state is valid.
   */
  AgentState(
    double x, double y, double z, double yaw, double vx, double vy, const AgentLabel & label,
    bool is_valid)
  : x(x), y(y), z(z), yaw(yaw), vx(vx), vy(vy), label(label), is_valid(is_valid)
  {
  }

  /**
   * @brief Construct a new AgentState object with ROS 2 message.
   *
   * @param position 3D position in map coordinate system.
   * @param yaw Yaw angle [rad] in map coordinate system.
   * @param velocity Velocity in map coordinate system.
   * @param label Label.
   * @param is_valid Indicates whther the state is valid.
   */
  AgentState(
    const geometry_msgs::msg::Point & position, double yaw,
    const geometry_msgs::msg::Vector3 & velocity, const AgentLabel & label, bool is_valid)
  : x(position.x),
    y(position.y),
    z(position.z),
    yaw(yaw),
    vx(velocity.x),
    vy(velocity.y),
    label(label),
    is_valid(is_valid)
  {
  }

  /**
   * @brief Return the number of state attributes, which is `8`.
   */
  static size_t num_attribute() { return 8; }

  double x{0.0};                          //!< Center x.
  double y{0.0};                          //!< Center y.
  double z{0.0};                          //!< Center z.
  double yaw{0.0};                        //!< Yaw angle [rad].
  double vx{0.0};                         //!< X-direction velocity [m/s].
  double vy{0.0};                         //!< Y-direction velocity [m/s].
  AgentLabel label{AgentLabel::UNKNOWN};  //!< Label.
  bool is_valid{false};                   //!< Indicates whether the state is valid.
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
   * @param num_past Number of past timestamps.
   * @param current_time Current timestamp.
   * @param state Current agent state.
   */
  AgentHistory(size_t num_past, double current_time, const value_type & state) : queue_(num_past)
  {
    update(current_time, state);
  }

  /**
   * @brief Construct a new AgentHistory object with an array of states.
   *
   * @param num_past Number of past timestamps.
   * @param timestamps Array of timestamps.
   * @param history Agent states at each timestamp.
   */
  AgentHistory(
    size_t num_past, const std::vector<double> & timestamps,
    const std::vector<value_type> & history)
  : queue_(num_past)
  {
    // validate inputs
    if (timestamps.size() != num_past) {
      std::ostringstream msg;
      msg << "Invalid size of timestamps: " << timestamps.size() << " != " << num_past;
      throw SimplException(SimplError_t::InvalidValue, msg.str());
    } else if (history.size() != num_past) {
      std::ostringstream msg;
      msg << "Invalid size of history: " << history.size() << " != " << num_past;
      throw SimplException(SimplError_t::InvalidValue, msg.str());
    }

    for (size_t t = 0; t < num_past; ++t) {
      const auto & state = history.at(t);
      if (state.is_valid) {
        update(timestamps.at(t), state);
      } else {
        update();
      }
    }
  }

  /**
   * @brief Update history with the latest state.
   *
   * @param current_time Current timestamp.
   * @param state Current agent state.
   */
  void update(double current_time, const value_type & state) noexcept
  {
    latest_time_ = current_time;
    queue_.push_back(state);
  }

  /**
   * @brief Update history with invalid state.
   *
   * Note that the latest valid timestamp is not updated.
   */
  void update() noexcept { queue_.push_back({}); }

  /**
   * @brief Check whether the latest valid state timestamp is too old or not.
   *
   * @param current_time Current timestamp.
   * @param threshold Time difference threshold value.
   */
  bool is_ancient(double current_time, double threshold) const
  {
    return std::abs(current_time - latest_time_) >= threshold;
  }

  /**
   * @brief Check whether the current state is valid or not.
   */
  bool is_current_valid() const noexcept { return current().is_valid; }

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

private:
  FixedQueue<value_type> queue_;  //!< Agent state container.
  double latest_time_;            //!< Latest valid timestamp.
};

/**
 * @brief A class to represent agent tensor data.
 */
class AgentTensor
{
public:
  /**
   * @brief Construct a new AgentTensor object.
   *
   * @param tensor 1D agent tensor data in the shape of (N*T*Da).
   * @param num_agent Number of agents (N).
   * @param num_past Number of past timestamps (T).
   * @param num_attribute Number of attributes (Da).
   */
  AgentTensor(
    const std::vector<float> & tensor, size_t num_agent, size_t num_past, size_t num_attribute)
  : num_agent(num_agent), num_past(num_past), num_attribute(num_attribute), tensor_(tensor)
  {
    if (tensor_.size() != num_agent * num_past * num_attribute) {
      std::ostringstream msg;
      msg << "Invalid size of agent tensor: " << tensor_.size()
          << " != " << num_agent * num_past * num_attribute;
      throw SimplException(SimplError_t::InvalidValue, msg.str());
    }
  }

  /**
   * @brief Return the pointer to the tensor data.
   */
  const float * data() const noexcept { return tensor_.data(); }

  /**
   * @brief Return the size of the tensor (N*T*Da).
   */
  size_t size() const noexcept { return tensor_.size(); }

  const size_t num_agent;      //!< Number of agents (N).
  const size_t num_past;       //!< Number of past timestamps (T).
  const size_t num_attribute;  //!< Number of attributes (Da).

private:
  std::vector<float> tensor_;  //!< Agent tensor data.
};
}  // namespace autoware::simpl::archetype
#endif  // AUTOWARE__SIMPL__ARCHETYPE__AGENT_HPP_
