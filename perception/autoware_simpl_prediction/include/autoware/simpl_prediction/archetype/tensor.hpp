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

#ifndef AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__TENSOR_HPP_
#define AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__TENSOR_HPP_

#include "autoware/simpl_prediction/archetype/exception.hpp"
#include "autoware/simpl_prediction/archetype/polyline.hpp"

#include <sstream>
#include <vector>

namespace autoware::simpl_prediction::archetype
{
/**
 * @brief A class to represent agent tensor data.
 */
class AgentTensor
{
public:
  /**
   * @brief Construct a new AgentTensor object.
   *
   * @param tensor 1D agent tensor data in the shape of (N*Da*T).
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
      throw SimplException(SimplError_t::INVALID_VALUE, msg.str());
    }
  }

  /**
   * @brief Return the pointer to the tensor data.
   */
  const float * data() const noexcept { return tensor_.data(); }

  /**
   * @brief Return the size of the tensor (N*Da*T).
   */
  size_t size() const noexcept { return tensor_.size(); }

  const size_t num_agent;      //!< Number of agents (N).
  const size_t num_past;       //!< Number of past timestamps (T).
  const size_t num_attribute;  //!< Number of attributes (Da).

private:
  std::vector<float> tensor_;  //!< Agent tensor data.
};

/**
 * @brief A class to represent map tensor data.
 */
class MapTensor
{
public:
  using size_type = std::vector<float>::size_type;

  /**
   * @brief Construct a new MapTensor object.
   *
   * @param tensor 1D map tensor data in the shape of (K*P*Dm).
   * @param num_polyline Number of polylines (K).
   * @param num_point Number of points contained in a single polyline (P).
   * @param num_attribute Number of attributes (Dm).
   */
  MapTensor(
    const std::vector<float> & tensor, size_t num_polyline, size_t num_point, size_t num_attribute)
  : num_polyline(num_polyline), num_point(num_point), num_attribute(num_attribute), tensor_(tensor)
  {
    if (tensor_.size() != num_polyline * num_point * num_attribute) {
      std::ostringstream msg;
      msg << "Invalid size of map tensor: " << tensor_.size()
          << " != " << num_polyline * num_point * num_attribute;
      throw SimplException(SimplError_t::INVALID_VALUE, msg.str());
    }
  }

  /**
   * @brief Return the pointer to tensor data.
   */
  const float * data() const noexcept { return tensor_.data(); }

  /**
   * @brief Return the size of tensor elements, where K*P*D.
   */
  size_type size() const noexcept { return tensor_.size(); }

  const size_t num_polyline;   //!< Number of polylines (K).
  const size_t num_point;      //!< Number of points contained in a single polyline (P).
  const size_t num_attribute;  //!< Number of attributes (Dm).

private:
  std::vector<float> tensor_;  //!< Map tensor data.
};
}  // namespace autoware::simpl_prediction::archetype
#endif  // AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__TENSOR_HPP_
