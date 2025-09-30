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

#ifndef AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__PREPROCESSING_UTILS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__PREPROCESSING_UTILS_HPP_

#include <Eigen/Core>

#include <geometry_msgs/msg/pose.hpp>

#include <cassert>
#include <deque>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{

using InputDataMap = std::unordered_map<std::string, std::vector<float>>;
using NormalizationMap =
  std::unordered_map<std::string, std::pair<std::vector<float>, std::vector<float>>>;

/**
 * @brief Normalizes the input data using the provided normalization map.
 *
 * This function iterates over the input data map and applies normalization
 * to each entry based on the corresponding parameters in the normalization map.
 * The normalization process typically scales or transforms the input data
 * to a standard range or distribution, which can improve the performance
 * and stability of downstream algorithms.
 *
 * @param[in,out] input_data_map      Map containing the input data to be normalized.
 * @param[in]     normalization_map   Map containing normalization parameters for each input.
 */
void normalize_input_data(
  InputDataMap & input_data_map, const NormalizationMap & normalization_map);

/**
 * @brief Creates ego agent past trajectory data from pose messages.
 *
 * This function processes a sequence of pose messages to create ego vehicle's
 * past trajectory data in the ego reference frame. Each timestep contains
 * x, y position and heading information as cos(yaw) and sin(yaw).
 *
 * @param[in] pose_msgs        Deque of pose messages
 * @param[in] num_timesteps       Number of timesteps to process
 * @param[in] map_to_ego_transform Transformation matrix from map to ego frame
 * @return Vector of floats containing [x, y, cos_yaw, sin_yaw] for each timestep
 */
std::vector<float> create_ego_agent_past(
  const std::deque<geometry_msgs::msg::Pose> & pose_msgs, size_t num_timesteps,
  const Eigen::Matrix4d & map_to_ego_transform);

/**
 * @brief Creates random sampled trajectories for diffusion model input.
 *
 * This function generates a set of random sampled trajectories based on the specified
 * temperature parameter. The trajectories are intended to be used as input for the diffusion
 * planner model.
 * @param[in] temperature Temperature parameter to control the randomness of the trajectories.
 * @return A vector of floats representing the sampled trajectories.
 */
std::vector<float> create_sampled_trajectories(const double temperature);

}  // namespace autoware::diffusion_planner::preprocess
#endif  // AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__PREPROCESSING_UTILS_HPP_
