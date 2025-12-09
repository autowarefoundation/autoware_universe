// Copyright 2025 Tier IV, Inc.
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

#include "autoware/planning_evaluator/obstacle_metrics_calculator.hpp"
#include "autoware/planning_evaluator/metrics/metrics_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <Eigen/Core>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <limits>
#include <vector>

namespace planning_diagnostics
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::calc_distance2d;
namespace bg = boost::geometry;

void ObstacleMetricsCalculator::setVehicleInfo(const VehicleInfo & vehicle_info)
{
  vehicle_info_ = vehicle_info;
}

void ObstacleMetricsCalculator::setPredictedObjects(const PredictedObjects & objects)
{
  predicted_objects_ = objects;
}

void ObstacleMetricsCalculator::setEgoPose(const nav_msgs::msg::Odometry & ego_odometry)
{
  ego_odometry_ = ego_odometry;
}

void ObstacleMetricsCalculator::setTrajectory(const Trajectory & traj)
{
  trajectory_ = traj;
}

void ObstacleMetricsCalculator::clearData()
{
  predicted_objects_.reset();
  ego_odometry_.reset();
  vehicle_info_.reset();
  trajectory_.reset();
  
  obstacle_distance_metrics_.clear();
  obstacle_ttc_metrics_.clear();
  obstacle_pet_metrics_.clear();
  obstacle_drac_metrics_.clear();
}

std::vector<std::pair<std::string, Accumulator<double>>> ObstacleMetricsCalculator::getMetric(
  const Metric metric) const
{
  switch (metric) {
    case Metric::obstacle_distance:
      return obstacle_distance_metrics_;
    case Metric::obstacle_ttc:
      return obstacle_ttc_metrics_;
    case Metric::obstacle_pet:
      return obstacle_pet_metrics_;
    case Metric::obstacle_drac:
      return obstacle_drac_metrics_;
    default:
      return {};
  }
}

bool ObstacleMetricsCalculator::isDataReady() const
{
  return trajectory_.has_value() && !trajectory_->points.empty() &&
         predicted_objects_.has_value() && !predicted_objects_->objects.empty() &&
         ego_odometry_.has_value() && vehicle_info_.has_value();
}

void ObstacleMetricsCalculator::PreprocessEgoTrajectory()
{
  ego_trajectory_points_.clear();
  ego_max_reachable_distance_ = 0.0;
  double vel_curr = ego_odometry_->twist.twist.linear.x;

  // Find closest point to ego pose and trim past trajectory
  size_t p0_index = 0;
  double last_dist_to_ego = calc_distance2d(ego_odometry_->pose.pose, trajectory_->points.front().pose);
  
  for (size_t i = 1; i < trajectory_->points.size(); ++i) {
    const double dist_to_ego = calc_distance2d(ego_odometry_->pose.pose, trajectory_->points.at(i).pose);
    if (dist_to_ego > last_dist_to_ego) {
      break;
    }
    last_dist_to_ego = dist_to_ego;
    p0_index = i;
  }

  // Insert first point at ego pose
  const bool initial_is_stop = (std::abs(vel_curr) <= parameters.stop_velocity_mps);
  ego_trajectory_points_.emplace_back(
    ego_odometry_->pose.pose, vel_curr, 0.0, 0.0, initial_is_stop);
  if (initial_is_stop || p0_index + 1 >= trajectory_->points.size()) {
    return;
  }

  // Process trajectory points
  double time_from_start_s = 0.0;
  double distance_from_start_m = 0.0;
  bool is_stop = false;

  for (size_t i = p0_index + 1; i < trajectory_->points.size(); ++i) {
    const auto & p_curr = trajectory_->points.at(i);
    const auto & trajectory_point_prev = ego_trajectory_points_.back();
    
    // Skip if segment distance is too small
    const double segment_dist = calc_distance2d(trajectory_point_prev.pose, p_curr.pose);
    if (segment_dist < parameters.min_spatial_interval_m) {
      continue;
    }

    // calculate new velocity if use ego trajectory velocity
    if (parameters.use_ego_traj_vel) {
      vel_curr = static_cast<double>(p_curr.longitudinal_velocity_mps);
      const double vel_lower_bound = std::sqrt(
        std::max(0.0, trajectory_point_prev.velocity_mps * trajectory_point_prev.velocity_mps + 
                 2.0 * parameters.limit_min_accel * segment_dist));
      vel_curr = vel_curr < 0 ? vel_curr : std::max(vel_curr, vel_lower_bound);

      double effective_velocity = (vel_curr + trajectory_point_prev.velocity_mps) * 0.5;
      effective_velocity = vel_curr >= 0 ? std::max(effective_velocity, 1e-2) : -std::min(-effective_velocity, -1e-2);
        
      const double segment_time = segment_dist / std::abs(effective_velocity);

      // Skip if segment time is too small
      if (segment_time < parameters.min_time_interval_s) {
        continue;
      }
    
      // Insert new ego trajectory point
      time_from_start_s += segment_time;
      distance_from_start_m += segment_dist;
      is_stop = (std::abs(vel_curr) <= parameters.stop_velocity_mps);

      ego_trajectory_points_.emplace_back(
        p_curr.pose, vel_curr, time_from_start_s, distance_from_start_m, is_stop);

    }
    else {
      const double segment_time = segment_dist / std::abs(vel_curr);
      if (segment_time < parameters.min_time_interval_s) {
        continue;
      }
      time_from_start_s += segment_time;
      distance_from_start_m += segment_dist;
      ego_trajectory_points_.emplace_back(
        p_curr.pose, vel_curr, time_from_start_s, distance_from_start_m, false);
    }

    // update max reachable distance
    const double dist_from_ego = calc_distance2d(ego_odometry_->pose.pose, p_curr.pose);
    ego_max_reachable_distance_ = std::max(ego_max_reachable_distance_, dist_from_ego);

    // if stopped, break
    if (is_stop) {
      break;
    }
  }
  
  // set polygon for all ego trajectory points
  for (auto & ego_trajectory_point : ego_trajectory_points_) {
    ego_trajectory_point.setPolygon(*vehicle_info_);
  }
}

void ObstacleMetricsCalculator::ProcessObstaclesTrajectory()
{
  obstacle_trajectory_points_.clear();
  obstacle_first_overlapping_index_.clear();
  obstacle_first_collision_index_.clear();
  obstacle_index_to_uuid_.clear();

  const auto & ego_initial_pose = ego_trajectory_points_.front().pose;
  const auto & ego_final_time = ego_trajectory_points_.back().time_from_start_s;

  // calculate ego margin
  const autoware_utils::LinearRing2d footprint = vehicle_info_->createFootprint();
  double ego_margin_max = 0.0;
  double ego_margin_min = std::numeric_limits<double>::max();
  for (const auto & point : footprint) {
    const double dist = std::sqrt(point.x() * point.x() + point.y() * point.y());
    ego_margin_max = std::max(ego_margin_max, dist);
    ego_margin_min = std::min(ego_margin_min, dist);
  }

  for (const auto & object : predicted_objects_->objects) {
    // 1. initialize
    const auto & obstacle_pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto & obstacle_twist = object.kinematics.initial_twist_with_covariance.twist.linear;
    const double obstacle_velocity = std::sqrt(obstacle_twist.x * obstacle_twist.x + obstacle_twist.y * obstacle_twist.y);
    const bool is_obstacle_stop = obstacle_velocity <= parameters.stop_velocity_mps;
    bool is_obstacle_traj_no_overlapping_ego_traj = false;

    // 2. roughly check if obstacle trajectory is no overlapping with ego trajectory.
    double obstacle_margin_max = 0.0;
    double obstacle_margin_min = std::numeric_limits<double>::max();
    const auto obstacle_polygon = autoware_utils::to_polygon2d(Pose(), object.shape);
    for (const auto & point : obstacle_polygon.outer()) {
      const double dist = std::sqrt(point.x() * point.x() + point.y() * point.y());
      obstacle_margin_max = std::max(obstacle_margin_max, dist);
      obstacle_margin_min = std::min(obstacle_margin_min, dist);
    }
    const double obstacle_max_reachable_distance = obstacle_velocity * ego_final_time;
    const double obstacle_ego_distance = calc_distance2d(ego_initial_pose, obstacle_pose);
    is_obstacle_traj_no_overlapping_ego_traj = (obstacle_ego_distance >= obstacle_max_reachable_distance + ego_max_reachable_distance_ + ego_margin_max + obstacle_margin_max + parameters.collision_thr_m);

    // 3. create obstacle trajectory points
    obstacle_trajectory_points_.emplace_back(obstacle_pose, obstacle_velocity, 0, 0, is_obstacle_stop);
    if (!is_obstacle_traj_no_overlapping_ego_traj && !is_obstacle_stop && !object.kinematics.predicted_paths.empty()) {
      const auto & obstacle_path = object.kinematics.predicted_paths.front().path;
  
      // initialize reference point and the point right after next reference point. Here `reference_point` is the point right before the current ego trajectory point.
      ObstacleTrajectoryPoint reference_point = obstacle_trajectory_points_.front();
      size_t next_reference_point_idx = 1;
      double next_reference_point_time_from_start_s = std::numeric_limits<double>::max();
      double next_reference_point_distance_from_start_m = std::numeric_limits<double>::max();
      double next_reference_point_time_from_start_s = std::numeric_limits<double>::max();

      if (next_reference_point_idx < obstacle_path.size()) {
        next_reference_point_distance_from_start_m = 
          calc_distance2d(reference_point.pose, obstacle_path[next_reference_point_idx]) + reference_point.distance_from_start_m;
        next_reference_point_time_from_start_s = next_reference_point_distance_from_start_m / obstacle_velocity;
      }
      
      for (size_t i = 1; i < ego_trajectory_points_.size(); ++i) {
        const auto & ego_trajectory_point = ego_trajectory_points_[i];
        const double ego_time = ego_trajectory_point.time_from_start_s;

        // Update the reference point and next reference point.
        while (next_reference_point_idx < obstacle_path.size() && 
               next_reference_point_time_from_start_s < ego_time) {

          reference_point = ObstacleTrajectoryPoint(
            obstacle_path[next_reference_point_idx],
            obstacle_velocity,
            next_reference_point_time_from_start_s,
            next_reference_point_distance_from_start_m,
            false);
          ++next_reference_point_idx;
          
          if (next_reference_point_idx < obstacle_path.size()) {
            const double next_reference_point_distance_from_start_m = 
              calc_distance2d(reference_point.pose, obstacle_path[next_reference_point_idx]) + reference_point.distance_from_start_m;
            next_reference_point_time_from_start_s = next_reference_point_distance_from_start_m / obstacle_velocity;
          }
        }

        // insert new obstacle trajectory point based on the reference point
        obstacle_trajectory_points_.emplace_back(reference_point, ego_time - reference_point.time_from_start_s);
      }
    }

    // 4. calculate `obstacle_distance` metrics
    if (metrics_need_[Metric::obstacle_distance]) {
      double obstacle_distance = std::numeric_limits<double>::max();

      const auto & first_obstacle_point = obstacle_trajectory_points_.front();
      first_obstacle_point.setPolygon(object.shape);
      const auto & first_obstacle_polygon = first_obstacle_point.polygon.value();

      for (auto & ego_trajectory_point : ego_trajectory_points_) {
        const auto dist = bg::distance(ego_trajectory_point.polygon.value(), first_obstacle_polygon);
        obstacle_distance = std::min(obstacle_distance, dist);
      }
      
      // add metric
      Accumulator<double> obstacle_distance_metric;
      obstacle_distance_metric.add(obstacle_distance);
      obstacle_distance_metrics_.emplace_back(utils::uuid_to_string(object.object_id), obstacle_distance_metric);
    }

    // 5. calculate `obstacle_ttc` metrics (TODO)

  }
}

void ObstacleMetricsCalculator::calculateMetrics()
{
  // Step 0. Check if data ready and pre-process trajectory
  if (!isDataReady()) {
    return;
  }
  PreprocessEgoTrajectory();
  ProcessObstaclesTrajectory();
}

}  // namespace planning_diagnostics