// Copyright 2025 TIER IV.
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

#include "../src/output_converter/trajectory_converter.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <cmath>
#include <map>
#include <vector>

namespace autoware::tensorrt_vad::vad_interface
{

OutputTrajectoryConverter::OutputTrajectoryConverter(
  const CoordinateTransformer & coordinate_transformer, const VadInterfaceConfig & config)
: Converter(coordinate_transformer, config)
{
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint>
OutputTrajectoryConverter::create_trajectory_points(
  const std::vector<float> & predicted_trajectory, const double trajectory_timestep,
  const Eigen::Matrix4d & base2map_transform) const
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;

  // function to transform direction vector from base coordinate system to map coordinate system
  auto transform_direction_to_map = [&base2map_transform](
                                      const float base_dx, const float base_dy) -> float {
    Eigen::Vector3d base_direction(static_cast<double>(base_dx), static_cast<double>(base_dy), 0.0);
    Eigen::Vector3d map_direction = base2map_transform.block<3, 3>(0, 0) * base_direction;
    return std::atan2(map_direction.y(), map_direction.x());
  };

  // Add 0-second point (0,0)
  autoware_planning_msgs::msg::TrajectoryPoint initial_point;
  Eigen::Vector4d init_ego_position = base2map_transform * Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
  initial_point.pose.position.x = init_ego_position[0];
  initial_point.pose.position.y = init_ego_position[1];
  initial_point.pose.position.z = init_ego_position[2];
  // Convert initial direction to map coordinate system (when facing x-direction in base coordinate
  // system)
  initial_point.pose.orientation =
    autoware_utils::create_quaternion_from_yaw(transform_direction_to_map(1.0f, 0.0f));
  initial_point.longitudinal_velocity_mps = 2.5;
  initial_point.lateral_velocity_mps = 0.0;
  initial_point.acceleration_mps2 = 0.0;
  initial_point.heading_rate_rps = 0.0;
  initial_point.time_from_start.sec = 0;
  initial_point.time_from_start.nanosec = 0;
  points.push_back(initial_point);

  double prev_x = init_ego_position[0];
  double prev_y = init_ego_position[1];
  auto prev_orientation = initial_point.pose.orientation;

  for (size_t i = 0; i < predicted_trajectory.size(); i += 2) {
    autoware_planning_msgs::msg::TrajectoryPoint point;

    float aw_x = predicted_trajectory[i];
    float aw_y = predicted_trajectory[i + 1];

    Eigen::Vector4d base_link_position(
      static_cast<double>(aw_x), static_cast<double>(aw_y), 0.0, 1.0);
    Eigen::Vector4d map_position = base2map_transform * base_link_position;

    point.pose.position.x = map_position[0];
    point.pose.position.y = map_position[1];
    point.pose.position.z = map_position[2];

    if (i + 2 < predicted_trajectory.size()) {
      float aw_dx = predicted_trajectory[i + 2] - predicted_trajectory[i];
      float aw_dy = predicted_trajectory[i + 3] - predicted_trajectory[i + 1];

      float yaw = transform_direction_to_map(aw_dx, aw_dy);
      point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    } else {
      point.pose.orientation = prev_orientation;
    }

    // Calculate velocity (divide distance from previous point by time interval)
    auto distance = std::hypot(point.pose.position.x - prev_x, point.pose.position.y - prev_y);
    point.longitudinal_velocity_mps = static_cast<float>(distance / trajectory_timestep);

    point.lateral_velocity_mps = 0.0;
    point.acceleration_mps2 = 0.0;
    point.heading_rate_rps = 0.0;

    // Set time_from_start (1 second, 2 seconds, 3 seconds, 4 seconds, 5 seconds, 6 seconds)
    size_t point_index = i / 2;
    double time_sec = (point_index + 1) * trajectory_timestep;
    point.time_from_start.sec = static_cast<int32_t>(time_sec);
    point.time_from_start.nanosec =
      static_cast<uint32_t>((time_sec - point.time_from_start.sec) * 1e9);

    // Save current position for next calculation
    prev_x = point.pose.position.x;
    prev_y = point.pose.position.y;
    prev_orientation = point.pose.orientation;

    points.push_back(point);
  }

  return points;
}

autoware_internal_planning_msgs::msg::CandidateTrajectories
OutputTrajectoryConverter::process_candidate_trajectories(
  const std::map<int32_t, std::vector<float>> & predicted_trajectories, const rclcpp::Time & stamp,
  const double trajectory_timestep, const Eigen::Matrix4d & base2map_transform) const
{
  autoware_internal_planning_msgs::msg::CandidateTrajectories candidate_trajectories_msg;

  // Add each command's trajectory as CandidateTrajectory
  for (const auto & [command_idx, trajectory] : predicted_trajectories) {
    autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_trajectory;

    // Set header
    candidate_trajectory.header.stamp = stamp;
    candidate_trajectory.header.frame_id = "map";

    // Set generator_id (unique UUID)
    candidate_trajectory.generator_id = autoware_utils_uuid::generate_uuid();

    candidate_trajectory.points =
      create_trajectory_points(trajectory, trajectory_timestep, base2map_transform);

    candidate_trajectories_msg.candidate_trajectories.push_back(candidate_trajectory);

    // Add GeneratorInfo for each command
    autoware_internal_planning_msgs::msg::GeneratorInfo generator_info;
    generator_info.generator_id = autoware_utils_uuid::generate_uuid();
    generator_info.generator_name.data = "autoware_tensorrt_vad_cmd_" + std::to_string(command_idx);
    candidate_trajectories_msg.generator_info.push_back(generator_info);
  }

  return candidate_trajectories_msg;
}

autoware_planning_msgs::msg::Trajectory OutputTrajectoryConverter::process_trajectory(
  const std::vector<float> & predicted_trajectory, const rclcpp::Time & stamp,
  const double trajectory_timestep, const Eigen::Matrix4d & base2map_transform) const
{
  autoware_planning_msgs::msg::Trajectory trajectory_msg;

  // Set header
  trajectory_msg.header.stamp = stamp;
  trajectory_msg.header.frame_id = "map";

  trajectory_msg.points =
    create_trajectory_points(predicted_trajectory, trajectory_timestep, base2map_transform);

  return trajectory_msg;
}

}  // namespace autoware::tensorrt_vad::vad_interface
