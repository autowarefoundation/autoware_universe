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

#ifndef AUTOWARE__SIMPL_PREDICTION__PROCESSING__POSTPROCESSOR_HPP_
#define AUTOWARE__SIMPL_PREDICTION__PROCESSING__POSTPROCESSOR_HPP_

#include "autoware/simpl_prediction/archetype/agent.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <std_msgs/msg/header.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::simpl_prediction::processing
{
/**
 * @brief A class for postprocessing.
 */
class PostProcessor
{
public:
  using Header = std_msgs::msg::Header;
  using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
  using PredictedObject = autoware_perception_msgs::msg::PredictedObject;
  using PredictedPath = autoware_perception_msgs::msg::PredictedPath;
  using TrackedObject = autoware_perception_msgs::msg::TrackedObject;
  using output_type = PredictedObjects;

  /**
   * @brief Construct a new PostProcessor object.
   *
   * @param num_mode Number of modes (M).
   * @param num_future Number of predicted future timestamps (Tf).
   * @param score_threshold Score threshold [0, 1].
   */
  PostProcessor(size_t num_mode, size_t num_future, double score_threshold);

  /**
   * @brief Execute postprocessing.
   *
   * @param scores Vector of scores [N'xM].
   * @param trajectories Vector of predicted trajectory attributes [N'xMxTfxDp].
   * @param agent_ids Agent IDs [N].
   */
  output_type process(
    const std::vector<float> & scores, const std::vector<float> & trajectories,
    const std::vector<std::string> & agent_ids, const Header & header,
    const std::unordered_map<std::string, TrackedObject> & tracked_object_map) const;

private:
  size_t num_mode_;         //!< Number of modes (M).
  size_t num_future_;       //!< Number of predicted future timestamps (Tf).
  double score_threshold_;  //!< Score threshold [0, 1].

  /**
   * @brief Sort mode with its score.
   *
   * @param scores Read only pointer to the first element of the mode score.
   * @return Indices of the sorted modes in descending order.
   */
  std::vector<size_t> sort_by_score(const float * scores) const;
};
}  // namespace autoware::simpl_prediction::processing
#endif  // AUTOWARE__SIMPL_PREDICTION__PROCESSING__POSTPROCESSOR_HPP_
