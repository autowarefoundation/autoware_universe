// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_ONEPLANNER__DETAIL_HPP_
#define AUTOWARE__TENSORRT_ONEPLANNER__DETAIL_HPP_

#include <autoware/diffusion_planner/dimensions.hpp>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

// Small, GPU-free helpers and constants shared by the OnePlanner node. Kept in a
// header (rather than file-local anonymous namespaces) so they can be unit-tested.
namespace autoware::tensorrt_oneplanner::detail
{

// Ego-trajectory StateNormalizer baked into the exported planner graph: positions
// are normalized as (value - mean) / std, with mean = (10, 0) m and std = (20, 20) m
// for (x, y). The warm-started (previous-frame) trajectory that is fed back into the
// `sampled_trajectories` input must be expressed in this same normalized space.
// These mirror the StateNormalizer ego statistics in OnePlanner's export_onnx.py.
inline constexpr float kEgoPositionXMean = 10.0f;
inline constexpr float kEgoPositionStd = 20.0f;

/// @brief Number of elements in a tensor shape, excluding the leading batch dimension.
/// Returns 1 for a shape with no non-batch dimensions (size <= 1), so an empty
/// container is well-defined rather than dereferencing past the end.
template <class Container>
std::size_t num_elements(const Container & shape)
{
  if (shape.size() <= 1) {
    return 1;
  }
  return std::accumulate(
    shape.begin() + 1, shape.end(), std::size_t{1}, std::multiplies<std::size_t>());
}

/// @brief Expand the ego-only prediction [OUTPUT_T, POSE_DIM] to the diffusion-planner
///        agent layout [MAX_NUM_AGENTS, OUTPUT_T, POSE_DIM] (neighbors zero-filled) so
///        the diffusion-planner postprocessing utilities can be reused unchanged.
/// @throws std::runtime_error if the input size is not exactly OUTPUT_T * POSE_DIM.
inline std::vector<float> expand_ego_prediction(const std::vector<float> & ego_prediction)
{
  const std::size_t agent_stride = static_cast<std::size_t>(autoware::diffusion_planner::OUTPUT_T) *
                                   static_cast<std::size_t>(autoware::diffusion_planner::POSE_DIM);
  if (ego_prediction.size() != agent_stride) {
    throw std::runtime_error(
      "Unexpected ego prediction size: " + std::to_string(ego_prediction.size()) + " (expected " +
      std::to_string(agent_stride) + ")");
  }
  std::vector<float> expanded(
    static_cast<std::size_t>(autoware::diffusion_planner::MAX_NUM_AGENTS) * agent_stride, 0.0f);
  std::copy(ego_prediction.begin(), ego_prediction.end(), expanded.begin());
  return expanded;
}

}  // namespace autoware::tensorrt_oneplanner::detail

#endif  // AUTOWARE__TENSORRT_ONEPLANNER__DETAIL_HPP_
