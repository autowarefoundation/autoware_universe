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
 * @brief Converts a vector of poses to a Trajectory message.
 *
 * @param poses The vector of 4x4 transformation matrices representing poses.
 * @param stamp The ROS time stamp for the message.
 * @param velocity_smoothing_window The window size for velocity smoothing.
 * @param enable_force_stop Whether to enable force stop logic.
 * @param stopping_threshold The threshold for keeping the stopping state [m/s].
 * @return A Trajectory message in map coordinates.
 */
Trajectory get_trajectory_from_poses(
  const std::vector<Eigen::Matrix4d> & poses, const rclcpp::Time & stamp,
  const int64_t velocity_smoothing_window, const bool enable_force_stop,
  const double stopping_threshold);
};  // namespace

std::vector<std::vector<std::vector<Eigen::Matrix4d>>> parse_predictions(
  const std::vector<float> & prediction)
{
  const int64_t batch_size = prediction.size() / (MAX_NUM_AGENTS * OUTPUT_T * POSE_DIM);

  // Ensure prediction has enough data
  const size_t required_size = batch_size * MAX_NUM_AGENTS * OUTPUT_T * POSE_DIM;
  if (prediction.size() < required_size) {
    throw std::runtime_error(
      "Prediction vector size (" + std::to_string(prediction.size()) +
      ") is smaller than required (" + std::to_string(required_size) + ")");
  }

  // Structure: batch -> agent -> timestep -> pose
  std::vector<std::vector<std::vector<Eigen::Matrix4d>>> parsed_predictions(
    batch_size,
    std::vector<std::vector<Eigen::Matrix4d>>(
      MAX_NUM_AGENTS, std::vector<Eigen::Matrix4d>(OUTPUT_T, Eigen::Matrix4d::Identity())));

  for (int64_t batch_idx = 0; batch_idx < batch_size; ++batch_idx) {
    for (int64_t agent_idx = 0; agent_idx < MAX_NUM_AGENTS; ++agent_idx) {
      for (int64_t time_idx = 0; time_idx < OUTPUT_T; ++time_idx) {
        const int64_t pred_base_idx =
          (batch_idx * MAX_NUM_AGENTS * OUTPUT_T + agent_idx * OUTPUT_T + time_idx) * POSE_DIM;

        const double x = static_cast<double>(prediction[pred_base_idx + 0]);
        const double y = static_cast<double>(prediction[pred_base_idx + 1]);
        const double cos_yaw = static_cast<double>(prediction[pred_base_idx + 2]);
        const double sin_yaw = static_cast<double>(prediction[pred_base_idx + 3]);

        // Create 4x4 transformation matrix from x, y, cos(yaw), sin(yaw)
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 0) = cos_yaw;
        pose(0, 1) = -sin_yaw;
        pose(1, 0) = sin_yaw;
        pose(1, 1) = cos_yaw;
        pose(0, 3) = x;
        pose(1, 3) = y;

        parsed_predictions[batch_idx][agent_idx][time_idx] = pose;
      }
    }
  }

  return parsed_predictions;
}

PredictedObjects create_predicted_objects(
  const std::vector<std::vector<std::vector<Eigen::Matrix4d>>> & agent_poses,
  const AgentData & ego_centric_agent_data, const rclcpp::Time & stamp,
  const Eigen::Matrix4d & transform_ego_to_map, const int64_t batch_index)
{
  auto trajectory_path_to_pose_path = [](const Trajectory & trajectory, const double object_z)
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

  // ego_centric_agent_data contains neighbor history information ordered by distance.
  for (int64_t neighbor_id = 0; neighbor_id < MAX_NUM_NEIGHBORS; ++neighbor_id) {
    if (static_cast<size_t>(neighbor_id) >= objects_history.size()) {
      break;
    }

    // Extract poses for this neighbor (neighbor_id + 1 because 0 is ego)
    std::vector<Eigen::Matrix4d> neighbor_poses;
    for (int64_t time_idx = 0; time_idx < OUTPUT_T; ++time_idx) {
      // Transform to map frame
      Eigen::Matrix4d pose_in_map =
        transform_ego_to_map * agent_poses[batch_index][neighbor_id + 1][time_idx];
      neighbor_poses.push_back(pose_in_map);
    }

    constexpr int64_t velocity_smoothing_window = 1;
    constexpr bool enable_force_stop = false;  // Don't force stop for neighbors
    constexpr double stopping_threshold = 0.0;
    const Trajectory trajectory_points_in_map_reference = get_trajectory_from_poses(
      neighbor_poses, stamp, velocity_smoothing_window, enable_force_stop, stopping_threshold);

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
  const std::vector<std::vector<std::vector<Eigen::Matrix4d>>> & agent_poses,
  const rclcpp::Time & stamp, const Eigen::Matrix4d & transform_ego_to_map,
  const int64_t batch_index, const int64_t velocity_smoothing_window, const bool enable_force_stop,
  const double stopping_threshold)
{
  const int64_t ego_index = 0;

  // Validate batch index
  if (batch_index < 0 || batch_index >= static_cast<int64_t>(agent_poses.size())) {
    throw std::out_of_range(
      "Invalid batch_index: " + std::to_string(batch_index) +
      ", batch_size=" + std::to_string(agent_poses.size()));
  }

  // Extract ego poses (ego_index = 0)
  std::vector<Eigen::Matrix4d> ego_poses;
  ego_poses.reserve(OUTPUT_T);
  for (int64_t time_idx = 0; time_idx < OUTPUT_T; ++time_idx) {
    // Transform to map frame
    Eigen::Matrix4d pose_in_map =
      transform_ego_to_map * agent_poses[batch_index][ego_index][time_idx];
    ego_poses.push_back(pose_in_map);
  }

  return get_trajectory_from_poses(
    ego_poses, stamp, velocity_smoothing_window, enable_force_stop, stopping_threshold);
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
Trajectory get_trajectory_from_poses(
  const std::vector<Eigen::Matrix4d> & poses, const rclcpp::Time & stamp,
  const int64_t velocity_smoothing_window, const bool enable_force_stop,
  const double stopping_threshold)
{
  Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = "map";
  constexpr double dt = 0.1;

  double prev_x = poses[0](0, 3);
  double prev_y = poses[0](1, 3);

  for (size_t i = 0; i < poses.size(); ++i) {
    TrajectoryPoint p;
    p.time_from_start.sec = static_cast<int>(dt * static_cast<double>(i));
    p.time_from_start.nanosec =
      static_cast<uint32_t>((dt * static_cast<double>(i) - p.time_from_start.sec) * 1e9);

    // Extract position from transformation matrix
    p.pose.position.x = poses[i](0, 3);
    p.pose.position.y = poses[i](1, 3);
    p.pose.position.z = poses[i](2, 3);

    // Extract 3x3 rotation matrix and convert to quaternion
    const Eigen::Matrix3d rotation_matrix = poses[i].block<3, 3>(0, 0);
    const Eigen::Quaterniond quaternion(rotation_matrix);
    p.pose.orientation.x = quaternion.x();
    p.pose.orientation.y = quaternion.y();
    p.pose.orientation.z = quaternion.z();
    p.pose.orientation.w = quaternion.w();

    auto distance = std::hypot(p.pose.position.x - prev_x, p.pose.position.y - prev_y);
    p.longitudinal_velocity_mps = static_cast<float>(distance / dt);

    prev_x = p.pose.position.x;
    prev_y = p.pose.position.y;
    trajectory.points.push_back(p);
  }

  // smooth velocity
  bool force_stop = false;
  const float threshold_velocity = static_cast<float>(stopping_threshold);
  const int64_t num_points = static_cast<int64_t>(poses.size());

  if (num_points <= velocity_smoothing_window) {
    throw std::invalid_argument("velocity_smoothing_window must be smaller than number of points");
  }

  for (int64_t i = 0; i + velocity_smoothing_window <= num_points; ++i) {
    double sum_velocity = 0.0;
    for (int64_t w = 0; w < velocity_smoothing_window; ++w) {
      sum_velocity += trajectory.points[i + w].longitudinal_velocity_mps;
    }
    trajectory.points[i].longitudinal_velocity_mps =
      static_cast<float>(sum_velocity / static_cast<double>(velocity_smoothing_window));

    // stopping logic
    if (
      enable_force_stop &&
      std::abs(trajectory.points[i].longitudinal_velocity_mps) < threshold_velocity) {
      force_stop = true;
    }
    if (i > 0 && force_stop) {
      trajectory.points[i].longitudinal_velocity_mps = 0.0f;
      trajectory.points[i].pose = trajectory.points[i - 1].pose;
    }
  }

  // keep the last smoothed velocity for the remaining points
  const auto last_smoothed_velocity =
    trajectory.points[num_points - velocity_smoothing_window].longitudinal_velocity_mps;
  for (int64_t i = num_points - velocity_smoothing_window + 1; i < num_points; ++i) {
    trajectory.points[i].longitudinal_velocity_mps = last_smoothed_velocity;
    if (force_stop) {
      trajectory.points[i].longitudinal_velocity_mps = 0.0f;
      trajectory.points[i].pose = trajectory.points[i - 1].pose;
    }
  }

  // calculate acceleration
  for (int64_t i = 0; i + 1 < num_points; ++i) {
    const double v0 = trajectory.points[i].longitudinal_velocity_mps;
    const double v1 = trajectory.points[i + 1].longitudinal_velocity_mps;
    trajectory.points[i].acceleration_mps2 = static_cast<float>((v1 - v0) / dt);
  }
  trajectory.points.back().acceleration_mps2 = 0.0f;

  return trajectory;
}

}  // namespace

}  // namespace autoware::diffusion_planner::postprocess
