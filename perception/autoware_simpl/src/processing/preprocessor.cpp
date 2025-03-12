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
#include "autoware/simpl/archetype/map.hpp"
#include "autoware/simpl/processing/geometry.hpp"
#include "autoware/simpl/processing/polyline.hpp"
#include "autoware/simpl/processing/rpe.hpp"

#include <math.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::simpl::processing
{
using output_type = PreProcessor::output_type;

PreProcessor::PreProcessor(
  const std::vector<size_t> & label_ids, size_t max_num_agent, size_t num_past,
  size_t max_num_polyline, size_t max_num_point, double break_distance)
: label_ids_(label_ids),
  max_num_agent_(max_num_agent),
  num_past_(num_past),
  max_num_polyline_(max_num_polyline),
  max_num_point_(max_num_point),
  break_distance_(break_distance)
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

  std::vector<float> in_tensor(max_num_agent_ * num_past_, num_attribute);
  NodePoints node_centers;
  NodePoints node_vectors;
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
        in_tensor[idx] = 0.0;
        in_tensor[idx + 1] = 0.0;
      } else {
        // XYt - XYt-1
        in_tensor[idx] = static_cast<float>(ret_state.x - history.at(t - 1).x);
        in_tensor[idx + 1] = static_cast<float>(ret_state.y - history.at(t - 1).y);
      }
      // cos, sin
      in_tensor[idx + 2] = static_cast<float>(std::cos(ret_state.yaw));
      in_tensor[idx + 3] = static_cast<float>(std::sin(ret_state.yaw));
      // vx, vy
      in_tensor[idx + 4] = static_cast<float>(ret_state.vx);
      in_tensor[idx + 5] = static_cast<float>(ret_state.vy);
      // onehot
      for (size_t l = 0; l < label_ids_.size(); ++l) {
        const auto & label_id = label_ids_.at(l);
        if (label_id == static_cast<size_t>(ret_state.label)) {
          in_tensor[idx + 6 + l] = 1.0f;
        } else {
          in_tensor[idx + 6 + l] = 0.0f;
        }
      }
      // is valid
      in_tensor[idx + 6 + num_label] = static_cast<float>(ret_state.is_valid);
    }

    // TODO(ktro2828): Use more reasonable operation. e.g. sorting by distance histories first.
    if (n >= max_num_agent_) {
      break;
    }
  }

  archetype::AgentTensor agent_tensor(in_tensor, max_num_agent_, num_past_, num_attribute);

  return {agent_tensor, node_centers, node_vectors};
}

MapMetadata PreProcessor::process_map(
  const archetype::MapPoints & map_points, const archetype::AgentState & current_ego) const noexcept
{
  // 1. create polylines
  auto polylines = create_polylines(map_points, current_ego, max_num_point_, break_distance_);

  // 2. Sort by distance to extract topK
  if (polylines.size() > max_num_polyline_) {
    std::sort(
      polylines.begin(), polylines.end(),
      [](const archetype::MapPoints & polyline1, const archetype::MapPoints & polyline2) {
        const auto distance1 = find_center(polyline1).distance();
        const auto distance2 = find_center(polyline2).distance();
        return distance1 < distance2;
      });
  }

  // 3. Create tensor, node centers and vectors
  constexpr size_t num_attribute = 4;
  std::vector<float> in_tensor(max_num_polyline_ * (max_num_point_ - 1) * num_attribute);
  NodePoints node_centers(max_num_polyline_);
  NodePoints node_vectors(max_num_polyline_);
  for (size_t i = 0; i < polylines.size() || i < max_num_polyline_; ++i) {
    const auto & polyline = polylines.at(i);

    const auto center = find_center(polyline);
    node_centers.at(i) = {center.x, center.y};

    const auto [vx, vy, _] = polyline.back().diff(polyline.front());
    const auto norm = std::hypot(vx, vy);
    node_vectors.at(i) = {vx / (norm + 1e-6), vy / (norm + 1e-6)};

    const auto theta = atan2(vy, vx);

    const auto first = transform2d(polyline.front(), vx, vy, theta);
    const auto last = transform2d(polyline.back(), vx, vy, theta);
    const auto [dx, dy, __] = last.diff(first);
    in_tensor.push_back(static_cast<float>(0.5 * (first.x + last.x)));
    in_tensor.push_back(static_cast<float>(0.5 * (first.y + last.y)));
    in_tensor.push_back(dx);
    in_tensor.push_back(dy);
  }

  archetype::MapTensor map_tensor(in_tensor, max_num_polyline_, max_num_point_, num_attribute);
  return {map_tensor, node_centers, node_vectors};
}

archetype::RpeTensor PreProcessor::process_rpe(
  const AgentMetadata & agent_metadata, const MapMetadata & map_metadata) const noexcept
{
  // Concatenate node centers and vectors of agent and map
  const size_t num_rpe = agent_metadata.size() + map_metadata.size();  // N + K
  constexpr size_t num_attribute = 5;                                  // D

  NodePoints node_centers, node_vectors;
  node_centers.reserve(num_rpe);
  node_vectors.reserve(num_rpe);
  for (size_t i = 0; i < agent_metadata.size(); ++i) {
    node_centers.emplace_back(agent_metadata.centers.at(i));
    node_vectors.emplace_back(agent_metadata.vectors.at(i));
  }
  for (size_t i = 0; i < map_metadata.size(); ++i) {
    node_centers.emplace_back(map_metadata.centers.at(i));
    node_vectors.emplace_back(map_metadata.vectors.at(i));
  }

  archetype::RpeTensor rpe_tensor;
  rpe_tensor.reserve(num_rpe * num_rpe * num_attribute);
  for (size_t i = 0; i < num_rpe; ++i) {
    const auto & ci = node_centers.at(i);
    const auto & vi = node_vectors.at(i);
    for (size_t j = 0; j < num_rpe; ++j) {
      const auto & cj = node_centers.at(j);
      const auto & vj = node_vectors.at(j);

      // position vector
      const double dvx = cj.x - ci.x;
      const double dvy = cj.y - ci.y;

      const double cos_a1 = cosine_pe(vj, vi);
      const double sin_a1 = sine_pe(vj, vi);
      const double cos_a2 = cosine_pe(vj, dvx, dvy);
      const double sin_a2 = sine_pe(vj, dvx, dvy);

      constexpr double rpe_radius = 100.0;  // NOTE: Referred to the original implementation
      const double distance = 2.0 * std::hypot(dvx, dvy) / rpe_radius;

      // (cos_a1, sin_a1, cos_a2, sin_a2, d)
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
