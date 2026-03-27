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

#include "autoware/trajectory_ranker/metrics/trajectory_consistency_metric.hpp"

#include <rclcpp/time.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::trajectory_ranker::metrics
{

void TrajectoryConsistency::setup_parameters()
{
  if (node()) {
    try {
      time_horizon_ = node()->declare_parameter<double>("trajectory_consistency.time_horizon", 2.0);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      time_horizon_ = node()->get_parameter("trajectory_consistency.time_horizon").as_double();
    }
  }
}

namespace
{
/**
 * @brief Transform points from world coordinates to ego vehicle frame
 * @param points Vector of points in world coordinates
 * @param ego_pose Current ego vehicle pose
 * @return Pair of vectors: first contains longitudinal coordinates, second contains lateral
 * coordinates
 */
std::pair<std::vector<double>, std::vector<double>> transform_to_ego_frame(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & ego_pose)
{
  std::vector<double> longitudinal_coords;
  std::vector<double> lateral_coords;

  if (points.empty()) {
    return {longitudinal_coords, lateral_coords};
  }

  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  const double cos_yaw = std::cos(ego_yaw);
  const double sin_yaw = std::sin(ego_yaw);

  longitudinal_coords.reserve(points.size());
  lateral_coords.reserve(points.size());

  for (const auto & point : points) {
    const double dx = point.x - ego_pose.position.x;
    const double dy = point.y - ego_pose.position.y;
    const double longitudinal = dx * cos_yaw + dy * sin_yaw;
    const double lateral = -dx * sin_yaw + dy * cos_yaw;
    longitudinal_coords.push_back(longitudinal);
    lateral_coords.push_back(lateral);
  }

  return {longitudinal_coords, lateral_coords};
}

/**
 * @brief Find trajectory point at specified time offset from trajectory start
 * @param trajectory Trajectory message with header timestamp
 * @param time_offset Time offset in seconds from trajectory start time
 * @return Point at specified time, or nullopt if not found
 */
std::optional<geometry_msgs::msg::Point> find_point_at_time(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double time_offset)
{
  if (trajectory.points.empty()) {
    return std::nullopt;
  }

  const double target_time = rclcpp::Time(trajectory.header.stamp).seconds() + time_offset;

  // Find the point closest to target_time
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const double point_time = rclcpp::Time(trajectory.header.stamp).seconds() +
                              rclcpp::Duration(trajectory.points[i].time_from_start).seconds();

    if (point_time >= target_time) {
      // Return this point (or interpolate if needed for better accuracy)
      return trajectory.points[i].pose.position;
    }
  }

  // If target_time is beyond trajectory end, return last point
  return trajectory.points.back().pose.position;
}

/**
 * @brief Calculate variance of coordinates
 * @param coords Vector of coordinate values
 * @return Variance [m^2]
 */
double calculate_variance(const std::vector<double> & coords)
{
  if (coords.size() < 2) {
    return 0.0;
  }

  // Calculate mean
  const double mean = std::accumulate(coords.begin(), coords.end(), 0.0) / coords.size();

  // Calculate variance
  double variance = 0.0;
  for (const auto & coord : coords) {
    const double diff = coord - mean;
    variance += diff * diff;
  }

  return variance / coords.size();
}
}  // namespace

void TrajectoryConsistency::evaluate(
  const std::shared_ptr<autoware::trajectory_ranker::DataInterface> & result,
  const float max_value) const
{
  if (!result->points() || result->points()->size() < 2) {
    std::vector<float> zero_metric(result->points() ? result->points()->size() : 1, 0.0f);
    result->set_metric(index(), zero_metric);
    return;
  }

  constexpr float epsilon = 1.0e-6f;
  if (max_value < epsilon) {
    std::vector<float> zero_metric(result->points()->size(), 0.0f);
    result->set_metric(index(), zero_metric);
    return;
  }

  // Get current ego pose from the first trajectory point
  const auto & ego_pose = result->points()->front().pose;

  // Time offset to extract point for consistency comparison
  const double time_offset_from_now = time_horizon_;
  // We want to compare where each trajectory predicted the vehicle would be
  // at a common future time. For each historical trajectory, we find the point that was predicted
  // for time T when that trajectory was generated. This allows us to measure consistency between
  // past and current predictions at the same future moment.
  const double current_time = rclcpp::Time(result->header().stamp).seconds();
  const double target_absolute_time = current_time + time_offset_from_now;

  // Collect trajectory points at target absolute time from historical trajectories
  std::vector<geometry_msgs::msg::Point> points_at_target_time;

  // Get trajectory history buffer
  const auto history = result->trajectory_history();

  if (history && !history->empty()) {
    // Collect points at target absolute time from all trajectories in the history buffer
    for (const auto & trajectory : *history) {
      const double trajectory_time = rclcpp::Time(trajectory.header.stamp).seconds();
      // Calculate required offset from this trajectory's start time to reach target absolute time
      const double required_offset = target_absolute_time - trajectory_time;

      // Only use trajectories where the required offset is positive and reasonable
      if (required_offset > 0.0) {
        auto point = find_point_at_time(trajectory, required_offset);
        if (point.has_value()) {
          points_at_target_time.push_back(point.value());
        }
      }
    }
  }

  // Add current trajectory's point at time_horizon_seconds ahead
  // Create a Trajectory message from current trajectory points
  autoware_planning_msgs::msg::Trajectory current_trajectory;
  current_trajectory.header = result->header();
  current_trajectory.points = *result->original();

  auto current_point = find_point_at_time(current_trajectory, time_offset_from_now);
  if (current_point.has_value()) {
    points_at_target_time.push_back(current_point.value());
  }

  // If we have less than 2 points, no consistency evaluation is possible
  if (points_at_target_time.size() < 2) {
    std::vector<float> zero_metric(result->points()->size(), 0.0f);
    result->set_metric(index(), zero_metric);
    return;
  }

  // Transform all points to ego frame once
  const auto [longitudinal_coords, lateral_coords] =
    transform_to_ego_frame(points_at_target_time, ego_pose);

  // Calculate both lateral and longitudinal variance
  const double lateral_variance = calculate_variance(lateral_coords);
  const double longitudinal_variance = calculate_variance(longitudinal_coords);

  // Combine lateral and longitudinal variance (total position variance)
  const double total_variance = std::sqrt(lateral_variance + longitudinal_variance);

  // Normalize variance to [0, 1] range using max_value as reference
  // Convert variance (m^2) to metric score
  const float normalized_variance = std::min(1.0f, static_cast<float>(total_variance) / max_value);

  // Apply this metric value to all points in the trajectory
  // (consistency is a trajectory-level metric, not point-wise)
  std::vector<float> consistency_metric(result->points()->size(), normalized_variance);

  result->set_metric(index(), consistency_metric);
}

}  // namespace autoware::trajectory_ranker::metrics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_ranker::metrics::TrajectoryConsistency,
  autoware::trajectory_ranker::metrics::MetricInterface)
