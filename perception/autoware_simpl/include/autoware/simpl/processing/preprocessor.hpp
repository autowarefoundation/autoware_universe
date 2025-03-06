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

#ifndef AUTOWARE__SIMPL__PROCESSING__PREPROCESSOR_HPP_
#define AUTOWARE__SIMPL__PROCESSING__PREPROCESSOR_HPP_

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/datatype.hpp"
#include "autoware/simpl/archetype/map.hpp"

#include <cstddef>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace autoware::simpl::processing
{
/**
 * @brief 2D point for node center and vector.
 */
struct NodePoint
{
  NodePoint() = default;
  NodePoint(double _x, double _y) : x(_x), y(_y) {}

  double x{0.0};
  double y{0.0};
};

using NodePoints = std::vector<NodePoint>;

/**
 * @brief Abstract base class of input metadata.
 */
struct AbstractMetadata
{
  using size_type = NodePoints::size_type;

  AbstractMetadata(const NodePoints & _centers, const NodePoints & _vectors)
  : centers(_centers), vectors(_vectors)
  {
    if (centers.size() != vectors.size()) {
      throw std::runtime_error("Size of node centers and vectors must be same.");
    }
  }

  size_type size() const noexcept { return centers.size(); }

  const NodePoints centers;  //!< Node centers, (X, 2).
  const NodePoints vectors;  //!< Node vectors, (X, 2).
};

/**
 * @brief Agent metadata containing input tensor and node data.
 */
struct AgentMetadata : public AbstractMetadata
{
  AgentMetadata(
    const archetype::AgentTensor & _tensor, const NodePoints & _centers,
    const NodePoints & _vectors)
  : AbstractMetadata(_centers, _vectors), tensor(_tensor)
  {
  }
  const archetype::AgentTensor tensor;  //!< Input tensor for agent data.
};

/**
 * @brief Map metadata containing input tensor and node data.
 */
struct MapMetadata : public AbstractMetadata
{
  MapMetadata(
    const archetype::MapTensor & _tensor, const NodePoints & _centers, const NodePoints & _vectors)
  : AbstractMetadata(_centers, _vectors), tensor(_tensor)
  {
  }

  const archetype::MapTensor tensor;  //!< Input tensor for map data.
};

/**
 * @brief A class to execute preprocessing.
 */
class PreProcessor
{
public:
  using output_type =
    std::tuple<archetype::AgentTensor, archetype::MapTensor, archetype::RpeTensor>;

  /**
   * @brief Construct a new Preprocessor object.
   *
   * @param label_ids Vector of predictable label ids.
   * @param max_num_agent Maximum number of predictable agents.
   * @param num_past Number of past timestamps.
   * @param max_num_polyline Maximum number of polylines.
   * @param max_num_point Maximum number of points in a single polyline.
   * @param break_distance Distance threshold to break two polylines.
   */
  PreProcessor(
    const std::vector<size_t> & label_ids, size_t max_num_agent, size_t num_past,
    size_t max_num_polyline, size_t max_num_point, double break_distance);

  /**
   * @brief Execute preprocessing.
   *
   * @param histories Vector of histories for each agent.
   * @param map_points Vector of map points.
   * @param ego_index Index of ego vehicle in `histories`.
   * @return output_type Returns `AgentTensor`, `MapTensor` and RPE tensor (`std::vector<float>`).
   */
  output_type process(
    const archetype::AgentHistories & histories, const archetype::MapPoints & map_points,
    size_t ego_index) const noexcept;

private:
  /**
   * @brief Execute preprocessing for agent tensor.
   *
   * @param histories Vector of agent histories.
   * @param current_ego Current ego state.
   */
  AgentMetadata process_agent(
    const archetype::AgentHistories & histories,
    const archetype::AgentState & current_ego) const noexcept;

  /**
   * @brief Execute preprocessing for map tensor.
   *
   * @param map_points Vector of map points.
   * @param current_ego Current ego state.
   */
  MapMetadata process_map(
    const archetype::MapPoints & map_points,
    const archetype::AgentState & current_ego) const noexcept;

  /**
   * @brief Execute preprocessing for RPE tensor (N+K*N+K*D).
   *
   * @param agent_metadata Processed agent data containing metadata.
   * @param current_ego Processed map data containing its metadata.
   */
  archetype::RpeTensor process_rpe(
    const AgentMetadata & agent_metadata, const MapMetadata & map_metadata) const noexcept;

  const std::vector<size_t> label_ids_;  //!< Vector of predictable label ids.
  const size_t max_num_agent_;           //!< Maximum number of predictable agents (N).
  const size_t num_past_;                //!< Number of past timestamps (Tp).
  const size_t max_num_polyline_;        //!< Maximum number of polylines (K).
  const size_t max_num_point_;           //!< Maximum number of points in a single polyline (P).
  const double break_distance_;          //!< Distance threshold to break two polylines.
};
}  // namespace autoware::simpl::processing
#endif  // AUTOWARE__SIMPL__PROCESSING__PREPROCESSOR_HPP_
