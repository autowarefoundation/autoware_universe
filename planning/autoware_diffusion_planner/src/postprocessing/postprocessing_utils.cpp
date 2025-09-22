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

#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/detail/predicted_objects__struct.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_planning_msgs::msg::TrajectoryPoint;

// internal functions
namespace
{
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
  const Eigen::Matrix4d & transform_matrix, Eigen::MatrixXd & output_matrix, int64_t column_idx,
  int64_t row_idx, bool do_translation = true);

/**
 * @brief Extracts tensor data from tensor prediction into an Eigen matrix.
 *
 * @param prediction The tensor prediction output.
 * @return An Eigen matrix containing the tensor data in row-major order.
 */
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> get_tensor_data(
  const std::vector<float> & prediction);

/**
 * @brief Converts a prediction matrix to a Trajectory message.
 *
 * @param prediction_matrix The prediction matrix for a single agent.
 * @param transform_ego_to_map The transformation matrix from ego to map coordinates.
 * @param stamp The ROS time stamp for the message.
 * @param velocity_smoothing_window The window size for velocity smoothing.
 * @return A Trajectory message in map coordinates.
 */
Trajectory get_trajectory_from_prediction_matrix(
  const Eigen::MatrixXd & prediction_matrix, const Eigen::Matrix4d & transform_ego_to_map,
  const rclcpp::Time & stamp, const int64_t velocity_smoothing_window);
};  // namespace

PredictedObjects create_predicted_objects(
  const std::vector<float> & prediction, const AgentData & ego_centric_agent_data,
  const rclcpp::Time & stamp, const Eigen::Matrix4d & transform_ego_to_map)
{
  auto trajectory_path_to_pose_path = [&](const Trajectory & trajectory, const double object_z)
    -> std::vector<geometry_msgs::msg::Pose> {
    std::vector<geometry_msgs::msg::Pose> pose_path;
    std::for_each(trajectory.points.begin(), trajectory.points.end(), [&](const auto & p) {
      auto object_pose = p.pose;
      object_pose.position.z = object_z;  // Set the z coordinate to the object's z
      pose_path.push_back(object_pose);
    });

    return pose_path;
  };

  const std::vector<autoware::diffusion_planner::AgentHistory> objects_history =
    ego_centric_agent_data.get_histories();

  PredictedObjects predicted_objects;
  predicted_objects.header.stamp = stamp;
  predicted_objects.header.frame_id = "map";

  constexpr double time_step{0.1};

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tensor_data =
    get_tensor_data(prediction);

  // ego_centric_agent_data contains neighbor history information ordered by distance.
  for (int64_t neighbor_id = 0; neighbor_id < MAX_NUM_NEIGHBORS; ++neighbor_id) {
    if (static_cast<size_t>(neighbor_id) >= objects_history.size()) {
      break;
    }
    // use batch 0
    constexpr int64_t batch_idx = 0;

    // Copy only the relevant part
    Eigen::MatrixXd prediction_matrix = tensor_data.block(
      batch_idx * MAX_NUM_AGENTS * OUTPUT_T + (neighbor_id + 1) * OUTPUT_T, 0, OUTPUT_T, POSE_DIM);
    prediction_matrix.transposeInPlace();
    postprocess::transform_output_matrix(transform_ego_to_map, prediction_matrix, 0, 0, true);
    postprocess::transform_output_matrix(transform_ego_to_map, prediction_matrix, 0, 2, false);
    prediction_matrix.transposeInPlace();

    constexpr int64_t velocity_smoothing_window = 1;
    const Trajectory trajectory_points_in_map_reference = get_trajectory_from_prediction_matrix(
      prediction_matrix, transform_ego_to_map, stamp, velocity_smoothing_window);

    PredictedObject object;
    const TrackedObject & object_info =
      objects_history.at(neighbor_id).get_latest_state().tracked_object();
    {  // Extract path from prediction
      PredictedPath predicted_path;
      const double object_pose_z = object_info.kinematics.pose_with_covariance.pose.position.z;

      predicted_path.path =
        trajectory_path_to_pose_path(trajectory_points_in_map_reference, object_pose_z);
      predicted_path.time_step = rclcpp::Duration::from_seconds(time_step);
      predicted_path.confidence = 1.0;
      object.kinematics.predicted_paths.push_back(predicted_path);
    }
    {  // Copy kinematics
      object.kinematics.initial_twist_with_covariance =
        object_info.kinematics.twist_with_covariance;
      object.kinematics.initial_acceleration_with_covariance =
        object_info.kinematics.acceleration_with_covariance;
      object.kinematics.initial_pose_with_covariance = object_info.kinematics.pose_with_covariance;
    }
    {  // Copy the remaining info
      object.object_id = object_info.object_id;
      object.classification = object_info.classification;
      object.shape = object_info.shape;
      object.existence_probability = object_info.existence_probability;
    }
    predicted_objects.objects.push_back(object);
  }
  return predicted_objects;
}

Trajectory create_ego_trajectory(
  const std::vector<float> & prediction, const rclcpp::Time & stamp,
  const Eigen::Matrix4d & transform_ego_to_map, const int64_t batch_index,
  const int64_t velocity_smoothing_window)
{
  const int64_t ego_index = 0;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tensor_data =
    get_tensor_data(prediction);

  // Validate indices before accessing block
  const int64_t start_row = batch_index * MAX_NUM_AGENTS * OUTPUT_T + ego_index * OUTPUT_T;
  if (start_row < 0 || start_row + OUTPUT_T > tensor_data.rows()) {
    throw std::out_of_range(
      "Invalid block access: start_row=" + std::to_string(start_row) +
      ", rows=" + std::to_string(OUTPUT_T) + ", tensor_rows=" + std::to_string(tensor_data.rows()));
  }

  // Extract and copy the block to ensure we have a proper matrix, not just a view
  Eigen::MatrixXd prediction_matrix = tensor_data.block(start_row, 0, OUTPUT_T, POSE_DIM).eval();
  prediction_matrix.transposeInPlace();
  postprocess::transform_output_matrix(transform_ego_to_map, prediction_matrix, 0, 0, true);
  postprocess::transform_output_matrix(transform_ego_to_map, prediction_matrix, 0, 2, false);
  prediction_matrix.transposeInPlace();

  return get_trajectory_from_prediction_matrix(
    prediction_matrix, transform_ego_to_map, stamp, velocity_smoothing_window);
}

TurnIndicatorsCommand create_turn_indicators_command(
  const std::vector<float> & turn_indicator_logit, const rclcpp::Time & stamp)
{
  TurnIndicatorsCommand turn_indicators_cmd;
  turn_indicators_cmd.stamp = stamp;

  // Apply softmax to convert logit to probabilities

  // Find the max value for numerical stability
  const float max_logit =
    *std::max_element(turn_indicator_logit.begin(), turn_indicator_logit.end());

  std::vector<float> probabilities(turn_indicator_logit.size());
  float sum = 0.0001f;  // Small value to avoid division by zero

  // Compute exp(logit - max_logit) for numerical stability
  for (size_t i = 0; i < turn_indicator_logit.size(); ++i) {
    probabilities[i] = std::exp(turn_indicator_logit[i] - max_logit);
    sum += probabilities[i];
  }

  // Normalize to get probabilities
  for (float & prob : probabilities) {
    prob /= sum;
  }

  // Find the class with highest probability
  const size_t max_idx = std::distance(
    probabilities.begin(), std::max_element(probabilities.begin(), probabilities.end()));
  turn_indicators_cmd.command = max_idx;

  return turn_indicators_cmd;
}

int64_t count_valid_elements(
  const std::vector<float> & data, int64_t len, int64_t dim2, int64_t dim3, int64_t batch_idx)
{
  const int64_t single_batch_size = len * dim2 * dim3;
  const int64_t batch_offset = batch_idx * single_batch_size;

  if (batch_offset + single_batch_size > static_cast<int64_t>(data.size()) || batch_idx < 0) {
    return 0;  // Invalid batch index or data size
  }

  int64_t valid_count = 0;
  const float epsilon = std::numeric_limits<float>::epsilon();

  // Iterate through each element in the len dimension for the specified batch
  for (int64_t i = 0; i < len; ++i) {
    bool is_valid_element = false;

    // Check all values in the (dim2, dim3) block for this element
    const int64_t element_offset = batch_offset + i * dim2 * dim3;
    for (int64_t j = 0; j < dim2 * dim3; ++j) {
      const int64_t idx = element_offset + j;
      if (std::abs(data[idx]) > epsilon) {
        is_valid_element = true;
        break;  // Found non-zero value, element is valid
      }
    }

    if (is_valid_element) {
      valid_count++;
    }
  }

  return valid_count;
}

namespace
{
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> get_tensor_data(
  const std::vector<float> & prediction)
{
  const int64_t batch = prediction.size() / (MAX_NUM_AGENTS * OUTPUT_T * POSE_DIM);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tensor_data(
    batch * MAX_NUM_AGENTS * OUTPUT_T, POSE_DIM);
  tensor_data.setZero();

  // Ensure prediction has enough data
  const size_t required_size = tensor_data.size();
  if (prediction.size() < required_size) {
    throw std::runtime_error(
      "Prediction vector size (" + std::to_string(prediction.size()) +
      ") is smaller than required (" + std::to_string(required_size) + ")");
  }

  for (size_t i = 0; i < required_size; ++i) {
    tensor_data.data()[i] = static_cast<double>(prediction[i]);
  }
  return tensor_data;
}

Trajectory get_trajectory_from_prediction_matrix(
  const Eigen::MatrixXd & prediction_matrix, const Eigen::Matrix4d & transform_ego_to_map,
  const rclcpp::Time & stamp, const int64_t velocity_smoothing_window)
{
  Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = "map";
  constexpr double dt = 0.1;
  Eigen::Vector4d ego_position = transform_ego_to_map * Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
  double prev_x = ego_position(0);
  double prev_y = ego_position(1);
  for (int64_t row = 0; row < prediction_matrix.rows(); ++row) {
    TrajectoryPoint p;
    p.time_from_start.sec = static_cast<int>(dt * static_cast<double>(row));
    p.time_from_start.nanosec =
      static_cast<int>((dt * static_cast<double>(row) - p.time_from_start.sec) * 1e9);
    p.pose.position.x = prediction_matrix(row, 0);
    p.pose.position.y = prediction_matrix(row, 1);
    p.pose.position.z = ego_position.z();
    auto yaw = std::atan2(prediction_matrix(row, 3), prediction_matrix(row, 2));
    yaw = static_cast<float>(autoware_utils::normalize_radian(yaw));
    p.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    auto distance = std::hypot(p.pose.position.x - prev_x, p.pose.position.y - prev_y);
    p.longitudinal_velocity_mps = static_cast<float>(distance / dt);

    prev_x = p.pose.position.x;
    prev_y = p.pose.position.y;
    trajectory.points.push_back(p);
  }

  // smooth velocity
  for (int64_t row = 0; row + velocity_smoothing_window <= prediction_matrix.rows(); ++row) {
    double sum_velocity = 0.0;
    for (int64_t w = 0; w < velocity_smoothing_window; ++w) {
      sum_velocity += trajectory.points[row + w].longitudinal_velocity_mps;
    }
    trajectory.points[row].longitudinal_velocity_mps =
      static_cast<float>(sum_velocity / static_cast<double>(velocity_smoothing_window));
  }

  // calculate acceleration
  for (int64_t row = 0; row + 1 < prediction_matrix.rows(); ++row) {
    const double v0 = trajectory.points[row].longitudinal_velocity_mps;
    const double v1 = trajectory.points[row + 1].longitudinal_velocity_mps;
    trajectory.points[row].acceleration_mps2 = static_cast<float>((v1 - v0) / dt);
  }
  trajectory.points.back().acceleration_mps2 = 0.0f;

  return trajectory;
}

void transform_output_matrix(
  const Eigen::Matrix4d & transform_matrix, Eigen::MatrixXd & output_matrix, int64_t column_idx,
  int64_t row_idx, bool do_translation)
{
  Eigen::Matrix<double, 4, OUTPUT_T> xy_block = Eigen::Matrix<double, 4, OUTPUT_T>::Zero();
  xy_block.block<2, OUTPUT_T>(0, 0) =
    output_matrix.block<2, OUTPUT_T>(row_idx, column_idx * OUTPUT_T);
  xy_block.row(3) = do_translation ? Eigen::Matrix<double, 1, OUTPUT_T>::Ones()
                                   : Eigen::Matrix<double, 1, OUTPUT_T>::Zero();

  Eigen::Matrix<double, 4, OUTPUT_T> transformed_block = transform_matrix * xy_block;
  output_matrix.block<2, OUTPUT_T>(row_idx, column_idx * OUTPUT_T) =
    transformed_block.block<2, OUTPUT_T>(0, 0);
}

}  // namespace

}  // namespace autoware::diffusion_planner::postprocess
