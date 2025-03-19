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

#include "autoware/simpl/archetype/map.hpp"
#include "autoware/simpl/archetype/polyline.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace autoware::simpl::processing
{
namespace
{
/////// polyline ///////
/**
 * @brief Break polylines.
 *
 * @param polylines
 * @param max_num_point
 * @param break_distance
 * @return std::vector<archetype::Polyline>
 */
std::vector<archetype::Polyline> break_polylines(
  const std::vector<archetype::Polyline> & polylines, size_t max_num_point, double break_distance)
{
  std::vector<archetype::Polyline> output;
  if (polylines.empty()) {
    return polylines;
  }

  std::vector<archetype::MapPoint> flattened;
  for (const auto & polyline : polylines) {
    for (const auto & point : polyline) {
      flattened.emplace_back(point);
    }
  }

  std::vector<archetype::MapPoint> buffer;
  buffer.emplace_back(flattened.front());

  for (size_t i = 1; i < flattened.size(); ++i) {
    const auto & previous = flattened[i - 1];
    const auto & current = flattened[i];

    const bool break_polyline =
      buffer.size() >= max_num_point || previous.distance_from(current) > break_distance;

    if (break_polyline) {
      output.emplace_back(buffer);
      buffer.clear();
    }

    buffer.emplace_back(current);
  }

  if (!buffer.empty()) {
    output.emplace_back(buffer);
  }

  return output;
}

/////// transform ///////

/**
 * @brief Transform a map point, which is `from`, to the coordinate frame of the specified pose.
 *
 * @param from Original map point.
 * @param to_x X location w.r.t other coordinate.
 * @param to_y Y location w.r.t other coordinate.
 * @param to_yaw Yaw angle w.r.t other coordinate.
 * @return archetype::MapPoint
 */
archetype::MapPoint transform2d(
  const archetype::MapPoint & from, double to_x, double to_y, double to_yaw)
{
  auto vcos = std::cos(to_yaw);
  auto vsin = std::sin(to_yaw);

  auto x = (from.x - to_x) * vcos + (from.y - to_y) * vsin;
  auto y = -(from.x - to_x) * vsin + (from.y - to_y) * vcos;

  return {x, y, from.z, from.label};
}

/////// positional encoding ///////

/**
 * @brief Perform cosine positional encoding.
 */
double cosine_pe(const NodePoint & v1, const NodePoint & v2)
{
  return (v1.x * v2.x + v1.y * v2.y) / std::clamp(v1.norm * v2.norm, 1e-6, 1e9);
}

/**
 * @brief Perform cosine positional encoding.
 */
double cosine_pe(const NodePoint & v1, const double v2_x, const double v2_y)
{
  const double v2_norm = std::hypot(v2_x, v2_y);
  return (v1.x * v2_x + v1.y * v2_y) / std::clamp(v1.norm * v2_norm, 1e-6, 1e9);
}

/**
 * @brief Perform sine positional encoding.
 */
double sine_pe(const NodePoint & v1, const NodePoint & v2)
{
  return (v1.x * v2.y - v1.y * v2.x) / std::clamp(v1.norm * v2.norm, 1e-6, 1e9);
}

/**
 * @brief Perform sine positional encoding.
 */
double sine_pe(const NodePoint & v1, const double v2_x, const double v2_y)
{
  const double v2_norm = std::hypot(v2_x, v2_y);
  return (v1.x * v2_y - v1.y * v2_x) / std::clamp(v1.norm * v2_norm, 1e-6, 1e9);
}
}  // namespace

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

PreProcessor::output_type PreProcessor::process(
  const std::vector<archetype::AgentHistory> & histories,
  const std::vector<archetype::Polyline> & polylines,
  const archetype::AgentState & current_ego) const
{
  const auto agent_metadata = this->process_agent(histories, current_ego);

  const auto map_metadata = this->process_map(polylines, current_ego);

  const auto rpe_tensor = this->process_rpe(agent_metadata, map_metadata);

  return {agent_metadata, map_metadata, rpe_tensor};
}

AgentMetadata PreProcessor::process_agent(
  const std::vector<archetype::AgentHistory> & histories,
  const archetype::AgentState & current_ego) const
{
  const size_t num_label = label_ids_.size();
  const size_t num_attribute = num_label + 7;  // L + 7

  std::vector<float> in_tensor(max_num_agent_ * num_past_ * num_attribute);
  std::vector<std::string> agent_ids;
  NodePoints node_centers(max_num_agent_);
  NodePoints node_vectors(max_num_agent_);
  // TODO(ktro2828): sort by distance and predict only agent where the current state is valid
  for (size_t n = 0; n < histories.size() && n < max_num_agent_; ++n) {
    const auto & history = histories.at(n);
    const auto & current = history.current();

    // extract current state
    agent_ids.emplace_back(history.agent_id);

    // retrieve node data
    const auto center = current.transform(current_ego);
    node_centers[n] = {center.x, center.y};
    node_vectors[n] = {std::cos(center.yaw), std::sin(center.yaw)};

    // transform from map coordinate to current state relative coordinate
    const auto transformed = history.transform_to_current();
    for (size_t t = 0; t < transformed.size(); ++t) {
      const auto & state = transformed.at(t);

      std::vector<float> attributes(num_attribute);
      // dx, dy
      if (t == 0) {
        // (0.0, 0.0) at t=0
        attributes[0] = 0.0;
        attributes[1] = 0.0;
      } else {
        // XYt - XYt-1
        const auto & previous = transformed.at(t - 1);
        attributes[0] = static_cast<float>(state.x - previous.x);
        attributes[1] = static_cast<float>(state.y - previous.y);
      }
      // cos, sin
      attributes[2] = static_cast<float>(std::cos(state.yaw));
      attributes[3] = static_cast<float>(std::sin(state.yaw));
      // vx, vy
      attributes[4] = static_cast<float>(state.vx);
      attributes[5] = static_cast<float>(state.vy);
      // onehot
      for (size_t l = 0; l < label_ids_.size(); ++l) {
        const auto & label_id = label_ids_.at(l);
        attributes[6 + l] = label_id == static_cast<size_t>(state.label) ? 1.0f : 0.0f;
      }
      // is valid
      attributes[num_attribute - 1] = static_cast<float>(state.is_valid);

      // assign transposed values (N*Da*T)
      for (size_t a = 0; a < num_attribute; ++a) {
        in_tensor[(n * num_attribute + a) * num_past_ + t] = attributes[a];
      }
    }
  }

  archetype::AgentTensor agent_tensor(in_tensor, max_num_agent_, num_past_, num_attribute);

  return {agent_tensor, agent_ids, node_centers, node_vectors};
}

MapMetadata PreProcessor::process_map(
  const std::vector<archetype::Polyline> & polylines,
  const archetype::AgentState & current_ego) const
{
  // w.r.t map coordinate frame
  auto result = break_polylines(polylines, max_num_point_, break_distance_);
  std::sort(
    result.begin(), result.end(),
    [&current_ego](const archetype::Polyline & p1, const archetype::Polyline & p2) {
      const auto d1 = p1.distance_from(current_ego);
      const auto d2 = p2.distance_from(current_ego);
      return d1 < d2;
    });

  // create tensor, node centers and vectors
  constexpr size_t num_attribute = 4;
  std::vector<float> in_tensor(max_num_polyline_ * max_num_point_ * num_attribute);
  NodePoints node_centers(max_num_polyline_);
  NodePoints node_vectors(max_num_polyline_);
  std::vector<archetype::Polyline> used_polylines;
  for (size_t k = 0; k < result.size() && k < max_num_polyline_; ++k) {
    // transform map to ego
    const auto & polyline = result.at(k).transform(current_ego);

    used_polylines.emplace_back(polyline);

    // node center
    const auto & center = polyline.center();
    node_centers[k] = {center.x, center.y};

    // node vector (normalized)
    const auto [nx, ny] = polyline.back().diff(polyline.front(), true);
    node_vectors[k] = {nx, ny};

    // tensor
    const auto theta = atan2(ny, nx);
    for (size_t p = 1; p < polyline.size(); ++p) {
      const auto current = transform2d(polyline.at(p), center.x, center.y, theta);
      const auto previous = transform2d(polyline.at(p - 1), center.x, center.y, theta);
      const auto [vx, vy] = current.diff(previous, false);

      const size_t idx = (k * max_num_point_ + (p - 1)) * num_attribute;
      in_tensor[idx] = static_cast<float>(0.5 * (current.x + previous.x));
      in_tensor[idx + 1] = static_cast<float>(0.5 * (current.y + previous.y));
      in_tensor[idx + 2] = static_cast<float>(vx);
      in_tensor[idx + 3] = static_cast<float>(vy);
    }
  }

  archetype::MapTensor map_tensor(
    in_tensor, max_num_polyline_, max_num_point_, num_attribute, used_polylines);
  return {map_tensor, node_centers, node_vectors};
}

PreProcessor::RpeTensor PreProcessor::process_rpe(
  const AgentMetadata & agent_metadata, const MapMetadata & map_metadata) const
{
  // Concatenate node centers and vectors of agent and map
  const size_t num_rpe = agent_metadata.size() + map_metadata.size();  // N + K
  constexpr size_t num_attribute = 5;                                  // D

  NodePoints node_centers(agent_metadata.centers.begin(), agent_metadata.centers.end());
  NodePoints node_vectors(agent_metadata.vectors.begin(), agent_metadata.vectors.end());
  node_centers.insert(node_centers.end(), map_metadata.centers.begin(), map_metadata.centers.end());
  node_vectors.insert(node_vectors.end(), map_metadata.vectors.begin(), map_metadata.vectors.end());

  RpeTensor rpe_tensor(num_rpe * num_rpe * num_attribute);
  for (size_t i = 0; i < num_rpe; ++i) {
    const auto & ci = node_centers.at(i);
    const auto & vi = node_vectors.at(i);
    for (size_t j = 0; j < num_rpe; ++j) {
      const auto & cj = node_centers.at(j);
      const auto & vj = node_vectors.at(j);

      // position vector
      const double dvx = cj.x - ci.x;
      const double dvy = cj.y - ci.y;

      const double cos_a1 = cosine_pe(vi, vj);
      const double sin_a1 = sine_pe(vi, vj);
      const double cos_a2 = cosine_pe(vi, dvx, dvy);
      const double sin_a2 = sine_pe(vi, dvx, dvy);

      constexpr double rpe_radius = 100.0;  // NOTE: Referred to the original implementation
      const double distance = 2.0 * std::hypot(dvx, dvy) / rpe_radius;

      // (cos_a1, sin_a1, cos_a2, sin_a2, d)
      const auto idx = (i * num_rpe + j) * num_attribute;
      rpe_tensor[idx] = static_cast<float>(cos_a1);
      rpe_tensor[idx + 1] = static_cast<float>(sin_a1);
      rpe_tensor[idx + 2] = static_cast<float>(cos_a2);
      rpe_tensor[idx + 3] = static_cast<float>(sin_a2);
      rpe_tensor[idx + 4] = static_cast<float>(distance);
    }
  }

  return rpe_tensor;
}
}  // namespace autoware::simpl::processing
