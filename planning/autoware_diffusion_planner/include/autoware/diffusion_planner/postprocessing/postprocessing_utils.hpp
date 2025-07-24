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
#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>
#include <autoware_perception_msgs/msg/detail/predicted_object__struct.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory__struct.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

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
using unique_identifier_msgs::msg::UUID;

/**
 * @brief Applies a transformation to a block of the output matrix.
 *
 * @param transform_matrix The transformation matrix to apply.
 * @param output_matrix The matrix to be transformed (in-place).
 * @param column_idx The column index of the block to transform.
 * @param row_idx The row index of the block to transform.
 * @param do_translation Whether to apply translation (true) or not (false).
 */
void transform_output_matrix(
  const Eigen::Matrix4f & transform_matrix, Eigen::MatrixXf & output_matrix, int64_t column_idx,
  int64_t row_idx, bool do_translation = true);

/**
 * @brief Extracts tensor data from tensor prediction into an Eigen matrix.
 *
 * @param prediction The tensor prediction output.
 * @return An Eigen matrix containing the tensor data in row-major order.
 */
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> get_tensor_data(
  const std::vector<float> & prediction);

/**
 * @brief Converts tensor prediction output to a prediction matrix in map coordinates.
 *
 * @param prediction The tensor prediction output.
 * @param transform_ego_to_map The transformation matrix from ego to map coordinates.
 * @param batch The batch index to extract.
 * @param agent The agent index to extract.
 * @return The prediction matrix for the specified batch and agent.
 */
Eigen::MatrixXf get_prediction_matrix(
  const std::vector<float> & prediction, const Eigen::Matrix4f & transform_ego_to_map,
  const int64_t batch = 0, const int64_t agent = 0);

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
  const rclcpp::Time & stamp, const Eigen::Matrix4f & transform_ego_to_map);
/**
 * @brief Converts a prediction matrix to a Trajectory message.
 *
 * @param prediction_matrix The prediction matrix for a single agent.
 * @param transform_ego_to_map The transformation matrix from ego to map coordinates.
 * @param stamp The ROS time stamp for the message.
 * @return A Trajectory message in map coordinates.
 */
Trajectory get_trajectory_from_prediction_matrix(
  const Eigen::MatrixXf & prediction_matrix, const Eigen::Matrix4f & transform_ego_to_map,
  const rclcpp::Time & stamp);

/**
 * @brief Creates a Trajectory message from tensor prediction for a specific batch and agent.
 *
 * @param prediction The tensor prediction output.
 * @param stamp The ROS time stamp for the message.
 * @param transform_ego_to_map The transformation matrix from ego to map coordinates.
 * @param batch The batch index to extract.
 * @param agent The agent index to extract.
 * @return A Trajectory message for the specified batch and agent.
 */
Trajectory create_trajectory(
  const std::vector<float> & prediction, const rclcpp::Time & stamp,
  const Eigen::Matrix4f & transform_ego_to_map, int64_t batch, int64_t agent);

/**
 * @brief Creates multiple Trajectory messages from tensor prediction for a range of batches and
 * agents.
 *
 * @param prediction The tensor prediction output.
 * @param stamp The ROS time stamp for the messages.
 * @param transform_ego_to_map The transformation matrix from ego to map coordinates.
 * @param start_batch The starting batch index.
 * @param start_agent The starting agent index.
 * @return A vector of Trajectory messages.
 */
std::vector<Trajectory> create_multiple_trajectories(
  const std::vector<float> & prediction, const rclcpp::Time & stamp,
  const Eigen::Matrix4f & transform_ego_to_map, int64_t start_batch, int64_t start_agent);

/**
 * @brief Converts a Trajectory message to a CandidateTrajectories message with generator info.
 *
 * @param trajectory The Trajectory message to convert.
 * @param generator_uuid The UUID of the trajectory generator.
 * @param generator_name The name of the trajectory generator.
 * @return A CandidateTrajectories message containing the input trajectory and generator info.
 */
CandidateTrajectories to_candidate_trajectories_msg(
  const Trajectory & trajectory, const UUID & generator_uuid, const std::string & generator_name);

}  // namespace autoware::diffusion_planner::postprocess
#endif  // AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_
