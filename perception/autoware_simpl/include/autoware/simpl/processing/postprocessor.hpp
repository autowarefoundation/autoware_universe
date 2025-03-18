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

#ifndef AUTOWARE__SIMPL__PROCESSING__POSTPROCESSOR_HPP_
#define AUTOWARE__SIMPL__PROCESSING__POSTPROCESSOR_HPP_

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/datatype.hpp"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace autoware::simpl::processing
{
/**
 * @brief A class for postprocessing.
 */
class PostProcessor
{
public:
  using output_type = archetype::Predictions;

  /**
   * @brief Construct a new PostProcessor object.
   *
   * @param num_mode Number of modes (M).
   * @param num_future Number of predicted future timestamps (Tf).
   */
  PostProcessor(size_t num_mode, size_t num_future);

  /**
   * @brief Execute postprocessing.
   *
   * @param scores Vector of scores [N'xM].
   * @param trajectories Vector of predicted trajectory attributes [N'xMxTfxDp].
   * @param current_states Agent ID and corresponding current state [N].
   */
  output_type process(
    const std::vector<float> & scores, const std::vector<float> & trajectories,
    const std::vector<std::pair<std::string, archetype::AgentState>> & current_states) const;

private:
  size_t num_mode_;    //!< Number of modes (M).
  size_t num_future_;  //!< Number of predicted future timestamps (Tf).
};
}  // namespace autoware::simpl::processing
#endif  // AUTOWARE__SIMPL__PROCESSING__POSTPROCESSOR_HPP_
