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

#ifndef AUTOWARE__SIMPL_PREDICTION__PROCESSING__PREPROCESSOR_HPP_
#define AUTOWARE__SIMPL_PREDICTION__PROCESSING__PREPROCESSOR_HPP_

#include "autoware/simpl_prediction/archetype/agent.hpp"
#include "autoware/simpl_prediction/archetype/exception.hpp"
#include "autoware/simpl_prediction/archetype/map.hpp"
#include "autoware/simpl_prediction/archetype/polyline.hpp"
#include "autoware/simpl_prediction/archetype/tensor.hpp"

#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::simpl_prediction::processing
{
/**
 * @brief 2D point for node center and vector.
 */
struct NodePoint
{
  NodePoint() = default;
  NodePoint(double _x, double _y) : x(_x), y(_y), norm(std::hypot(_x, _y)), is_valid(true) {}

  double x{0.0};
  double y{0.0};
  double norm{0.0};
  bool is_valid{false};
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
      throw archetype::SimplException(
        archetype::SimplError_t::INVALID_VALUE, "Size of node centers and vectors must be same.");
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
    const archetype::AgentTensor & _tensor, const std::vector<std::string> & _agent_ids,
    const NodePoints & _centers, const NodePoints & _vectors)
  : AbstractMetadata(_centers, _vectors), tensor(_tensor), agent_ids(_agent_ids)
  {
  }
  const archetype::AgentTensor tensor;       //!< Input tensor for agent data.
  const std::vector<std::string> agent_ids;  //!< Agent IDs.
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
  using RpeTensor = std::vector<float>;  //!< Relative pose encoding tensor
  using output_type = std::tuple<AgentMetadata, MapMetadata, RpeTensor>;

  /**
   * @brief Construct a new Preprocessor object.
   *
   * @param label_ids Vector of predictable label ids.
   * @param max_num_agent Maximum number of predictable agents.
   * @param num_past Number of past timestamps.
   * @param max_num_polyline Maximum number of polylines.
   * @param max_num_point Maximum number of points in a single polyline.
   * @param polyline_range_distance Distance threshold from ego to trim polylines [m].
   * @param polyline_break_distance Distance threshold to break two polylines [m].
   */
  PreProcessor(
    const std::vector<size_t> & label_ids, size_t max_num_agent, size_t num_past,
    size_t max_num_polyline, size_t max_num_point, double polyline_range_distance,
    double polyline_break_distance);

  /**
   * @brief Execute preprocessing.
   *
   * @param histories Hasmap of histories for each agent ID.
   * @param polylines Vector of polylines.
   * @param current_ego Current ego state.
   * @return output_type Returns `AgentTensor`, `MapTensor` and RPE tensor (`std::vector<float>`).
   */
  output_type process(
    const std::vector<archetype::AgentHistory> & histories,
    const std::vector<archetype::Polyline> & polylines,
    const archetype::AgentState & current_ego) const;

private:
  /**
   * @brief Execute preprocessing for agent tensor.
   *
   * @param histories Hasmap of histories for each agent ID.
   * @param current_ego Current ego state.
   */
  AgentMetadata process_agent(
    const std::vector<archetype::AgentHistory> & histories,
    const archetype::AgentState & current_ego) const;

  /**
   * @brief Execute preprocessing for map tensor.
   *
   * @param polylines Vector of polylines.
   * @param current_ego Current ego state.
   */
  MapMetadata process_map(
    const std::vector<archetype::Polyline> & polylines,
    const archetype::AgentState & current_ego) const;

  /**
   * @brief Execute preprocessing for RPE (Relative Pose Encoding) tensor (N+K*N+K*D).
   *
   * @param agent_metadata Processed agent data containing metadata.
   * @param current_ego Processed map data containing its metadata.
   */
  RpeTensor process_rpe(
    const AgentMetadata & agent_metadata, const MapMetadata & map_metadata) const;

  const std::vector<size_t> label_ids_;   //!< Vector of predictable label ids.
  const size_t max_num_agent_;            //!< Maximum number of predictable agents (N).
  const size_t num_past_;                 //!< Number of past timestamps (Tp).
  const size_t max_num_polyline_;         //!< Maximum number of polylines (K).
  const size_t max_num_point_;            //!< Maximum number of points in a single polyline (P).
  const double polyline_range_distance_;  //!< Distance threshold from ego to trim polylines [m].
  const double polyline_break_distance_;  //!< Distance threshold to break two polylines [m].
};
}  // namespace autoware::simpl_prediction::processing
#endif  // AUTOWARE__SIMPL_PREDICTION__PROCESSING__PREPROCESSOR_HPP_
