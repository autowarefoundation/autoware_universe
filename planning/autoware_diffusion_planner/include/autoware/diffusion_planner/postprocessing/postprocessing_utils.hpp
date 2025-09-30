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

#ifndef AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_

#include "autoware/diffusion_planner/conversion/agent.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <cassert>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using unique_identifier_msgs::msg::UUID;

/**
 * @brief Creates PredictedObjects message from tensor prediction and agent data.
 *
 * @param prediction The tensor prediction output.
 * @param ego_centric_agent_data The agent data in ego-centric coordinates.
 * @param stamp The ROS time stamp for the message.
 * @param transform_ego_to_map The transformation matrix from ego to map coordinates.
 * @return A PredictedObjects message containing predicted paths for each agent.
 */
PredictedObjects create_predicted_objects(
  const std::vector<float> & prediction, const AgentData & ego_centric_agent_data,
  const rclcpp::Time & stamp, const Eigen::Matrix4d & transform_ego_to_map);

/**
 * @brief Creates a Trajectory message from tensor prediction for a specific batch and agent.
 *
 * @param prediction The tensor prediction output.
 * @param stamp The ROS time stamp for the message.
 * @param transform_ego_to_map The transformation matrix from ego to map coordinates.
 * @param batch_index The batch index to extract.
 * @param velocity_smoothing_window The window size for velocity smoothing.
 * @return A Trajectory message for the specified batch and agent.
 */
Trajectory create_ego_trajectory(
  const std::vector<float> & prediction, const rclcpp::Time & stamp,
  const Eigen::Matrix4d & transform_ego_to_map, const int64_t batch_index,
  const int64_t velocity_smoothing_window);

/**
 * @brief Converts turn indicator logit to TurnIndicatorsCommand message.
 *
 * @param turn_indicator_logit The turn indicator logit from the model output.
 * @param stamp The ROS time stamp for the message.
 * @return A TurnIndicatorsCommand message with the predicted turn indicators.
 */
TurnIndicatorsCommand create_turn_indicators_command(
  const std::vector<float> & turn_indicator_logit, const rclcpp::Time & stamp);

/**
 * @brief Counts valid elements in a tensor with shape (B, len, dim2, dim3).
 * An element is considered valid if not all values in the (dim2, dim3) block are zero.
 *
 * @param data The input tensor data (flattened).
 * @param len The length dimension.
 * @param dim2 The second-to-last dimension.
 * @param dim3 The last dimension.
 * @param batch_idx The batch index to examine (0-based).
 * @return The number of valid elements in the specified batch.
 */
int64_t count_valid_elements(
  const std::vector<float> & data, int64_t len, int64_t dim2, int64_t dim3, int64_t batch_idx);

}  // namespace autoware::diffusion_planner::postprocess
#endif  // AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_
