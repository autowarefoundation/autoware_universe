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

#include <autoware_internal_planning_msgs/msg/detail/candidate_trajectory__builder.hpp>
#include <autoware_internal_planning_msgs/msg/detail/candidate_trajectory__struct.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>
#include <autoware_perception_msgs/msg/detail/predicted_objects__struct.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_planning_msgs::msg::TrajectoryPoint;

void transform_output_matrix(
  const Eigen::Matrix4f & transform_matrix, Eigen::MatrixXf & output_matrix, int64_t column_idx,
  int64_t row_idx, bool do_translation)
{
  Eigen::Matrix<float, 4, OUTPUT_T> xy_block = Eigen::Matrix<float, 4, OUTPUT_T>::Zero();
  xy_block.block<2, OUTPUT_T>(0, 0) =
    output_matrix.block<2, OUTPUT_T>(row_idx, column_idx * OUTPUT_T);
  xy_block.row(3) = do_translation ? Eigen::Matrix<float, 1, OUTPUT_T>::Ones()
                                   : Eigen::Matrix<float, 1, OUTPUT_T>::Zero();

  Eigen::Matrix<float, 4, OUTPUT_T> transformed_block = transform_matrix * xy_block;
  output_matrix.block<2, OUTPUT_T>(row_idx, column_idx * OUTPUT_T) =
    transformed_block.block<2, OUTPUT_T>(0, 0);
};

PredictedObjects create_predicted_objects(
  const std::vector<float> & prediction, const AgentData & ego_centric_agent_data,
  const rclcpp::Time & stamp, const Eigen::Matrix4f & transform_ego_to_map)
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

  const auto objects_history = ego_centric_agent_data.get_histories();

  PredictedObjects predicted_objects;
  predicted_objects.header.stamp = stamp;
  predicted_objects.header.frame_id = "map";

  constexpr double time_step{0.1};
  constexpr auto prediction_shape = OUTPUT_SHAPE;
  constexpr auto agent_size = prediction_shape[1];

  // get agent trajectories excluding ego (start from batch 0, and agent 1)
  constexpr int64_t start_batch = 0;
  constexpr int64_t start_agent = 1;

  auto agent_trajectories =
    create_multiple_trajectories(prediction, stamp, transform_ego_to_map, start_batch, start_agent);

  // First prediction is of ego (agent 0). Predictions from index 1 to last are of the closest
  // neighbors. ego_centric_agent_data contains neighbor history information ordered by distance.
  for (int64_t agent = 1; agent < agent_size; ++agent) {
    if (static_cast<size_t>(agent) - 1 >= objects_history.size()) {
      break;
    }
    PredictedObject object;
    const auto & object_info = objects_history.at(agent - 1).get_latest_state().tracked_object();
    {  // Extract path from prediction
      const auto & trajectory_points_in_map_reference = agent_trajectories.at(agent - 1);
      PredictedPath predicted_path;
      const auto object_pose_z = object_info.kinematics.pose_with_covariance.pose.position.z;

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

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> get_tensor_data(
  const std::vector<float> & prediction)
{
  // copy relevant part of data to Eigen matrix
  constexpr auto prediction_shape = OUTPUT_SHAPE;

  auto batch_size = prediction_shape[0];
  auto agent_size = prediction_shape[1];
  auto rows = prediction_shape[2];
  auto cols = prediction_shape[3];

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tensor_data(
    batch_size * agent_size * rows, cols);
  tensor_data.setZero();

  // Ensure prediction has enough data
  const size_t required_size = tensor_data.size();
  if (prediction.size() < required_size) {
    throw std::runtime_error(
      "Prediction vector size (" + std::to_string(prediction.size()) +
      ") is smaller than required (" + std::to_string(required_size) + ")");
  }

  std::memcpy(tensor_data.data(), prediction.data(), required_size * sizeof(float));
  return tensor_data;
}

Eigen::MatrixXf get_prediction_matrix(
  const std::vector<float> & prediction, const Eigen::Matrix4f & transform_ego_to_map,
  const int64_t batch, const int64_t agent)
{
  // TODO(Daniel): add batch support
  const auto prediction_shape = OUTPUT_SHAPE;

  // copy relevant part of data to Eigen matrix
  auto agent_size = prediction_shape[1];
  auto rows = prediction_shape[2];
  auto cols = prediction_shape[3];

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tensor_data =
    get_tensor_data(prediction);
  // Validate indices before accessing block
  const int64_t start_row = batch * agent_size * rows + agent * rows;
  if (start_row < 0 || start_row + rows > tensor_data.rows()) {
    throw std::out_of_range(
      "Invalid block access: start_row=" + std::to_string(start_row) +
      ", rows=" + std::to_string(rows) + ", tensor_rows=" + std::to_string(tensor_data.rows()));
  }

  // Extract and copy the block to ensure we have a proper matrix, not just a view
  Eigen::MatrixXf prediction_matrix = tensor_data.block(start_row, 0, rows, cols).eval();

  // Copy only the relevant part
  prediction_matrix.transposeInPlace();
  postprocess::transform_output_matrix(transform_ego_to_map, prediction_matrix, 0, 0, true);
  postprocess::transform_output_matrix(transform_ego_to_map, prediction_matrix, 0, 2, false);
  return prediction_matrix.transpose();
}

Trajectory get_trajectory_from_prediction_matrix(
  const Eigen::MatrixXf & prediction_matrix, const Eigen::Matrix4f & transform_ego_to_map,
  const rclcpp::Time & stamp)
{
  Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = "map";
  // TODO(Daniel): check there is no issue with the speed of 1st point (index 0)
  constexpr double dt = 0.1f;
  Eigen::Vector4f ego_position = transform_ego_to_map * Eigen::Vector4f(0.0, 0.0, 0.0, 1.0);
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
  return trajectory;
}

Trajectory create_trajectory(
  const std::vector<float> & prediction, const rclcpp::Time & stamp,
  const Eigen::Matrix4f & transform_ego_to_map, int64_t batch, int64_t agent)
{
  // one batch of prediction
  Eigen::MatrixXf prediction_matrix =
    get_prediction_matrix(prediction, transform_ego_to_map, batch, agent);
  return get_trajectory_from_prediction_matrix(prediction_matrix, transform_ego_to_map, stamp);
}

std::vector<Trajectory> create_multiple_trajectories(
  const std::vector<float> & prediction, const rclcpp::Time & stamp,
  const Eigen::Matrix4f & transform_ego_to_map, int64_t start_batch, int64_t start_agent)
{
  constexpr auto prediction_shape = OUTPUT_SHAPE;
  constexpr auto batch_size = prediction_shape[0];
  constexpr auto agent_size = prediction_shape[1];
  constexpr auto rows = prediction_shape[2];
  constexpr auto cols = prediction_shape[3];

  std::vector<Trajectory> agent_trajectories;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tensor_data =
    get_tensor_data(prediction);

  for (int64_t batch = start_batch; batch < batch_size; ++batch) {
    for (int64_t agent = start_agent; agent < agent_size; ++agent) {
      // Copy only the relevant part
      Eigen::MatrixXf prediction_matrix =
        tensor_data.block(batch * agent_size * rows + agent * rows, 0, rows, cols);

      prediction_matrix.transposeInPlace();
      postprocess::transform_output_matrix(transform_ego_to_map, prediction_matrix, 0, 0, true);
      postprocess::transform_output_matrix(transform_ego_to_map, prediction_matrix, 0, 2, false);
      prediction_matrix.transposeInPlace();
      agent_trajectories.push_back(
        get_trajectory_from_prediction_matrix(prediction_matrix, transform_ego_to_map, stamp));
    }
  }
  return agent_trajectories;
}

CandidateTrajectories to_candidate_trajectories_msg(
  const Trajectory & trajectory, const UUID & generator_uuid, const std::string & generator_name)
{
  const auto candidate_trajectory = autoware_internal_planning_msgs::build<
                                      autoware_internal_planning_msgs::msg::CandidateTrajectory>()
                                      .header(trajectory.header)
                                      .generator_id(generator_uuid)
                                      .points(trajectory.points);

  std_msgs::msg::String generator_name_msg;
  generator_name_msg.data = generator_name;

  const auto generator_info =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::GeneratorInfo>()
      .generator_id(generator_uuid)
      .generator_name(generator_name_msg);

  const auto output = autoware_internal_planning_msgs::build<
                        autoware_internal_planning_msgs::msg::CandidateTrajectories>()
                        .candidate_trajectories({candidate_trajectory})
                        .generator_info({generator_info});
  return output;
}

}  // namespace autoware::diffusion_planner::postprocess
