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

// cspell: ignore onehot

#include "autoware/simpl/processing/preprocessor.hpp"

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/datatype.hpp"
#include "autoware/simpl/archetype/map.hpp"
#include "autoware/simpl/processing/geometry.hpp"
#include "autoware/simpl/processing/rpe.hpp"

#include <tuple>
#include <utility>
#include <vector>

namespace autoware::simpl::processing
{
using output_type = PreProcessor::output_type;

PreProcessor::PreProcessor(
  const std::vector<size_t> & label_ids, size_t max_num_agent, size_t num_past,
  size_t max_num_polyline, size_t max_num_point)
: label_ids_(label_ids),
  max_num_agent_(max_num_agent),
  num_past_(num_past),
  max_num_polyline_(max_num_polyline),
  max_num_point_(max_num_point)
{
}

output_type PreProcessor::process(
  const archetype::AgentHistories & histories, const archetype::MapPoints & map_points,
  size_t ego_index) const noexcept
{
  const auto & current_ego = histories.at(ego_index).current();

  const auto agent_metadata = this->process_agent(histories, current_ego);

  const auto map_metadata = this->process_map(map_points, current_ego);

  const auto rpe_tensor = this->process_rpe(agent_metadata, map_metadata);

  return {agent_metadata.tensor, map_metadata.tensor, rpe_tensor};
}

AgentMetadata PreProcessor::process_agent(
  const archetype::AgentHistories & histories,
  const archetype::AgentState & current_ego) const noexcept
{
  const size_t num_label = label_ids_.size();
  const size_t num_attribute = num_label + 7;  // L + 7

  std::vector<float> input_tensor(max_num_agent_ * num_past_, num_attribute);
  archetype::NodePoints node_centers;
  archetype::NodePoints node_vectors;
  for (size_t n = 0; n < histories.size(); ++n) {
    const auto & history = histories.at(n);
    const auto & current_state = history.current();

    // Retrieve node data
    const auto center = transform2d(current_state, current_ego);
    node_centers.emplace_back(center.x, center.y);
    node_vectors.emplace_back(std::cos(center.yaw), std::sin(center.yaw));
    for (size_t t = 0; t < history.size(); ++t) {
      const auto & state_t = history.at(t);

      // Transform from map coordinate to current state relative coordinate
      const auto ret_state = transform2d(state_t, current_state);

      const auto idx = (n * num_past_ + t) * num_attribute;

      // dx, dy
      if (t == 0) {
        // (0.0, 0.0) at t=0
        input_tensor[idx] = 0.0;
        input_tensor[idx + 1] = 0.0;
      } else {
        // XYt - XYt-1
        input_tensor[idx] = static_cast<float>(ret_state.x - history.at(t - 1).x);
        input_tensor[idx + 1] = static_cast<float>(ret_state.y - history.at(t - 1).y);
      }
      // cos, sin
      input_tensor[idx + 2] = static_cast<float>(std::cos(ret_state.yaw));
      input_tensor[idx + 3] = static_cast<float>(std::sin(ret_state.yaw));
      // vx, vy
      input_tensor[idx + 4] = static_cast<float>(ret_state.vx);
      input_tensor[idx + 5] = static_cast<float>(ret_state.vy);
      // onehot
      for (size_t l = 0; l < label_ids_.size(); ++l) {
        const auto & label_id = label_ids_.at(l);
        if (label_id == static_cast<size_t>(ret_state.label)) {
          input_tensor[idx + 6 + l] = 1.0f;
        } else {
          input_tensor[idx + 6 + l] = 0.0f;
        }
      }
      // is valid
      input_tensor[idx + 6 + num_label] = static_cast<float>(ret_state.is_valid);
    }

    // TODO(ktro2828): Implement much reasonable operation.
    if (n >= max_num_agent_) {
      break;
    }
  }

  archetype::AgentTensor agent_tensor(input_tensor, max_num_agent_, num_past_, num_attribute);

  return {agent_tensor, node_centers, node_vectors};
}

// TODO(ktro2828): Implement map processing.
// MapMetadata PreProcessor::process_map(
//   const archetype::MapPoints & map_points, const archetype::AgentState & current_ego) const
//   noexcept
// {
//   // 1. create polylines & extract topK
//   std::vector<float> input_tensor;
//   archetype::NodePoints node_centers;
//   archetype::NodePoints node_vectors;
//   for (size_t i = 1; i < map_points.size(); ++i) {
//   }
// }

archetype::RpeTensor PreProcessor::process_rpe(
  const AgentMetadata & agent_metadata, const MapMetadata & map_metadata) const noexcept
{
  // Concatenate node centers and vectors of agent and map
  archetype::NodePoints node_centers;
  archetype::NodePoints node_vectors;
  for (size_t i = 0; i < agent_metadata.size(); ++i) {
    node_centers.emplace_back(agent_metadata.centers.at(i));
    node_vectors.emplace_back(agent_metadata.vectors.at(i));
  }
  for (size_t i = 0; i < map_metadata.size(); ++i) {
    node_centers.emplace_back(map_metadata.centers.at(i));
    node_vectors.emplace_back(map_metadata.vectors.at(i));
  }

  const size_t num_rpe = agent_metadata.size() + map_metadata.size();  // N + K

  archetype::RpeTensor rpe_tensor;
  for (size_t i = 0; i < num_rpe; ++i) {
    const auto & center_i = node_centers.at(i);
    const auto & vector_i = node_vectors.at(i);
    for (size_t j = 0; j < num_rpe; ++j) {
      const auto & center_j = node_centers.at(i);
      const auto & vector_j = node_vectors.at(j);

      // v_pos
      const double vx = center_j.x - center_i.x;
      const double vy = center_j.y - center_i.y;

      const double cos_a1 = encode_cosine(vector_j, vector_i);
      const double sin_a1 = encode_sine(vector_j, vector_i);
      const double cos_a2 = encode_cosine(vector_j, vx, vy);
      const double sin_a2 = encode_sine(vector_j, vx, vy);

      constexpr double rpe_radius = 100.0;
      const double distance = 2.0 * std::hypot(vx, vy) / rpe_radius;

      rpe_tensor.push_back(static_cast<float>(cos_a1));
      rpe_tensor.push_back(static_cast<float>(sin_a1));
      rpe_tensor.push_back(static_cast<float>(cos_a2));
      rpe_tensor.push_back(static_cast<float>(sin_a2));
      rpe_tensor.push_back(static_cast<float>(distance));
    }
  }

  return rpe_tensor;
}
}  // namespace autoware::simpl::processing
