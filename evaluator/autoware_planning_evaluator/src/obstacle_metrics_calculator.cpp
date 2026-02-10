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

#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace planning_diagnostics
{
using autoware_planning_msgs::msg::TrajectoryPoint;

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
  trajectory_.reset();

  for (auto & [metric, metrics_vector] : obstacle_metrics_) {
    metrics_vector.clear();
  }
}

std::vector<std::pair<std::string, Accumulator<double>>> ObstacleMetricsCalculator::getMetric(
  const Metric metric) const
{
  const auto it = obstacle_metrics_.find(metric);
  if (it != obstacle_metrics_.end()) {
    return it->second;
  }
  return {};
}

void ObstacleMetricsCalculator::setMetricNeed(const Metric metric, bool need)
{
  // Only allow setting for obstacle metrics
  for (const auto obstacle_metric : obstacle_metric_types) {
    if (obstacle_metric == metric) {
      metrics_need_[metric] = need;
      return;
    }
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
  double last_dist_to_ego =
    calc_distance2d(ego_odometry_->pose.pose, trajectory_->points.front().pose);

  for (size_t i = 1; i < trajectory_->points.size(); ++i) {
    const double dist_to_ego =
      calc_distance2d(ego_odometry_->pose.pose, trajectory_->points.at(i).pose);
    if (dist_to_ego > last_dist_to_ego) {
      break;
    }
    last_dist_to_ego = dist_to_ego;
    p0_index = i;
  }

  // Insert first point at ego pose
  ego_trajectory_points_.emplace_back(ego_odometry_->pose.pose, vel_curr, 0.0, 0.0);

  // Process trajectory points
  double time_from_start_s = 0.0;
  double distance_from_start_m = 0.0;

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
        std::max(
          0.0, trajectory_point_prev.velocity_mps * trajectory_point_prev.velocity_mps +
                 2.0 * parameters.limit_min_accel * segment_dist));
      vel_curr = vel_curr < 0 ? vel_curr : std::max(vel_curr, vel_lower_bound);

      const double effective_velocity =
        std::abs(vel_curr + trajectory_point_prev.velocity_mps) * 0.5;

      // Use infinity for trajectory points after the ego stops
      const double segment_time = effective_velocity > parameters.stop_velocity_mps
                                    ? segment_dist / effective_velocity
                                    : std::numeric_limits<double>::infinity();

      // Skip if segment time is too small
      if (segment_time < parameters.min_time_interval_s) {
        continue;
      }

      // Insert new ego trajectory point
      time_from_start_s += segment_time;  // NOTE: it can be infinity, which means there is a stop
                                          // point before this ego trajectory point.
      distance_from_start_m += segment_dist;
      ego_trajectory_points_.emplace_back(
        p_curr.pose, vel_curr, time_from_start_s, distance_from_start_m);

    } else {
      const double segment_time = segment_dist / std::abs(vel_curr);
      if (segment_time < parameters.min_time_interval_s) {
        continue;
      }
      time_from_start_s += segment_time;
      distance_from_start_m += segment_dist;
      ego_trajectory_points_.emplace_back(
        p_curr.pose, vel_curr, time_from_start_s, distance_from_start_m);
    }

    // update max reachable distance if the ego trajectory point is before the first stop point.
    if (std::isfinite(time_from_start_s)) {
      const double dist_from_ego = calc_distance2d(ego_odometry_->pose.pose, p_curr.pose);
      ego_max_reachable_distance_ = std::max(ego_max_reachable_distance_, dist_from_ego);
    }
  }

  // set polygon for all ego trajectory points
  for (auto & ego_trajectory_point : ego_trajectory_points_) {
    ego_trajectory_point.setPolygon(*vehicle_info_);
  }
}

void ObstacleMetricsCalculator::ProcessObstaclesTrajectory()
{
  // calculate ego margin using the first ego trajectory point's pose and polygon
  if (ego_trajectory_points_.empty()) {
    return;
  }

  const auto & first_ego_point = ego_trajectory_points_.front();

  const auto [ego_margin_min, ego_margin_max] =
    metrics::utils::calculate_point_to_polygon_boundary_distances(
      first_ego_point.pose, first_ego_point.polygon.value());

  for (const auto & object : predicted_objects_->objects) {
    // ------------------------------------------------------------------------------------------------
    // 1. initialize
    obstacle_trajectory_points_.clear();

    const auto & obstacle_pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto & obstacle_twist = object.kinematics.initial_twist_with_covariance.twist.linear;
    const double obstacle_velocity =
      std::sqrt(obstacle_twist.x * obstacle_twist.x + obstacle_twist.y * obstacle_twist.y);
    bool is_obstacle_traj_no_overlapping_ego_traj = false;

    std::string object_uuid = metrics::utils::uuid_to_string(object.object_id);
    ;
    // ------------------------------------------------------------------------------------------------
    // 2. roughly check if obstacle trajectory is no overlapping with ego trajectory.

    const auto obstacle_polygon = autoware_utils::to_polygon2d(obstacle_pose, object.shape);
    const auto [obstacle_margin_min, obstacle_margin_max] =
      metrics::utils::calculate_point_to_polygon_boundary_distances(
        obstacle_pose, obstacle_polygon);

    const auto & ego_initial_pose = ego_trajectory_points_.front().pose;
    const auto & ego_final_time = ego_trajectory_points_.back().time_from_start_s;

    const double obstacle_max_reachable_distance = obstacle_velocity * ego_final_time;
    const double obstacle_ego_distance = calc_distance2d(ego_initial_pose, obstacle_pose);
    is_obstacle_traj_no_overlapping_ego_traj =
      (obstacle_ego_distance >= obstacle_max_reachable_distance + ego_max_reachable_distance_ +
                                  ego_margin_max + obstacle_margin_max +
                                  parameters.collision_thr_m);

    // ------------------------------------------------------------------------------------------------
    // 3. create obstacle trajectory points:
    //  - It creates the same number of points as the ego trajectory points with the same
    //  `time_from_start_s`.
    //  - But if there is stop point in the ego trajectory, obstacle trajectory points after that
    //  timing are not created.

    obstacle_trajectory_points_.emplace_back(obstacle_pose, obstacle_velocity, 0, 0);
    if (!is_obstacle_traj_no_overlapping_ego_traj) {
      if (object.kinematics.predicted_paths.empty()) {
        // Create duplicate obstacle trajectory points with same `time_from_start_s` as ego
        // trajectory points.
        for (size_t i = 1; i < ego_trajectory_points_.size(); ++i) {
          const double ego_time = ego_trajectory_points_[i].time_from_start_s;
          if (!std::isfinite(ego_time)) {
            break;
          }
          obstacle_trajectory_points_.emplace_back(obstacle_pose, obstacle_velocity, ego_time, 0.0);
        }
      } else {
        // Create obstacle trajectory points based on the predicted path with highest confidence.
        const auto & predicted_paths = object.kinematics.predicted_paths;
        auto max_confidence_iter = std::max_element(
          predicted_paths.begin(), predicted_paths.end(),
          [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
        const auto & obstacle_path = max_confidence_iter->path;

        // initialize reference point and the point right after next reference point. Here
        // `reference_point` is the point right before the current ego trajectory point.
        ObstacleTrajectoryPoint reference_point = obstacle_trajectory_points_.front();
        size_t next_reference_point_idx = 1;
        double next_reference_point_time_from_start_s = std::numeric_limits<double>::max();
        double next_reference_point_distance_from_start_m = std::numeric_limits<double>::max();

        if (next_reference_point_idx < obstacle_path.size()) {
          next_reference_point_distance_from_start_m =
            calc_distance2d(reference_point.pose, obstacle_path[next_reference_point_idx]) +
            reference_point.distance_from_start_m;
          next_reference_point_time_from_start_s =
            next_reference_point_distance_from_start_m / obstacle_velocity;
        }

        // create obstacle trajectory points based on the corresponding ego trajectory points.
        for (size_t i = 1; i < ego_trajectory_points_.size(); ++i) {
          const auto & ego_trajectory_point = ego_trajectory_points_[i];
          const double ego_time = ego_trajectory_point.time_from_start_s;

          if (!std::isfinite(ego_trajectory_point.time_from_start_s)) {
            break;
          }

          // Update the reference point and next reference point.
          while (next_reference_point_idx < obstacle_path.size() &&
                 next_reference_point_time_from_start_s < ego_time) {
            reference_point = ObstacleTrajectoryPoint(
              obstacle_path[next_reference_point_idx], obstacle_velocity,
              next_reference_point_time_from_start_s, next_reference_point_distance_from_start_m);

            ++next_reference_point_idx;

            if (next_reference_point_idx < obstacle_path.size()) {
              next_reference_point_distance_from_start_m =
                calc_distance2d(reference_point.pose, obstacle_path[next_reference_point_idx]) +
                reference_point.distance_from_start_m;
              next_reference_point_time_from_start_s =
                next_reference_point_distance_from_start_m / obstacle_velocity;
            }
          }

          // insert new obstacle trajectory point based on the reference point and
          // `time_from_start_s` of the corresponding ego trajectory point.
          obstacle_trajectory_points_.emplace_back(
            reference_point, ego_time - reference_point.time_from_start_s);
        }
      }
    }

    // ------------------------------------------------------------------------------------------------
    // 4. calculate `obstacle_distance` metrics

    if (metrics_need_[Metric::obstacle_distance]) {
      double obstacle_distance = std::numeric_limits<double>::max();
      const auto & first_obstacle_point = obstacle_trajectory_points_.front();

      for (auto & ego_trajectory_point : ego_trajectory_points_) {
        const double dist = calc_distance2d(ego_trajectory_point.pose, first_obstacle_point.pose);
        obstacle_distance = std::min(obstacle_distance, dist);
      }

      // add to metric statistics
      Accumulator<double> obstacle_distance_metric;
      obstacle_distance_metric.add(obstacle_distance);
      obstacle_metrics_[Metric::obstacle_distance].emplace_back(
        object_uuid, obstacle_distance_metric);
    }

    // ------------------------------------------------------------------------------------------------
    // 5. check overlap and collision between ego trajectory and obstacle trajectory

    for (size_t i = 0; i < obstacle_trajectory_points_.size(); ++i) {
      auto & obstacle_trajectory_point = obstacle_trajectory_points_[i];

      for (size_t j = 0; j < ego_trajectory_points_.size(); ++j) {
        const auto & ego_trajectory_point = ego_trajectory_points_[j];

        if (!std::isfinite(ego_trajectory_point.time_from_start_s)) {
          break;
        }

        const double dist =
          calc_distance2d(ego_trajectory_point.pose, obstacle_trajectory_point.pose);
        bool is_overlapping = false;
        if (dist <= (ego_margin_min + obstacle_margin_min + parameters.collision_thr_m)) {
          is_overlapping = true;
        } else if (dist > ego_margin_max + obstacle_margin_max + parameters.collision_thr_m) {
          is_overlapping = false;
        } else {
          obstacle_trajectory_point.setPolygon(object.shape);
          is_overlapping = metrics::utils::polygon_intersects(
            ego_trajectory_point.polygon.value(), obstacle_trajectory_point.polygon.value());
        }

        if (is_overlapping) {
          obstacle_trajectory_point.is_overlapping_with_ego_trajectory = true;
          obstacle_trajectory_point.is_collision_with_ego_trajectory =
            obstacle_trajectory_point.is_collision_with_ego_trajectory ||
            std::abs(
              obstacle_trajectory_point.time_from_start_s -
              ego_trajectory_point.time_from_start_s) < 1e-6;
          obstacle_trajectory_point.first_overlapping_ego_trajectory_index =
            std::min(obstacle_trajectory_point.first_overlapping_ego_trajectory_index, j);
          obstacle_trajectory_point.last_overlapping_ego_trajectory_index =
            std::max(obstacle_trajectory_point.last_overlapping_ego_trajectory_index, j);
        }
      }
    }

    // ------------------------------------------------------------------------------------------------
    // 6. get `obstacle_ttc` metrics

    if (metrics_need_[Metric::obstacle_ttc]) {
      for (auto & obstacle_trajectory_point : obstacle_trajectory_points_) {
        if (obstacle_trajectory_point.is_collision_with_ego_trajectory) {
          Accumulator<double> obstacle_ttc_metric;
          obstacle_ttc_metric.add(obstacle_trajectory_point.time_from_start_s);
          obstacle_metrics_[Metric::obstacle_ttc].emplace_back(object_uuid, obstacle_ttc_metric);
          break;
        }
      }
    }

    // ------------------------------------------------------------------------------------------------
    // 7. get `obstacle_pet` metrics

    if (metrics_need_[Metric::obstacle_pet]) {
      double obstacle_pet = std::numeric_limits<double>::max();
      for (size_t i = 0; i < obstacle_trajectory_points_.size(); ++i) {
        const auto & obstacle_trajectory_point = obstacle_trajectory_points_[i];
        if (obstacle_trajectory_point.is_overlapping_with_ego_trajectory) {
          const size_t ego_first_overlap_idx =
            obstacle_trajectory_point.first_overlapping_ego_trajectory_index;
          if (ego_first_overlap_idx == 0) {  // skip if overlap at t=0 (pet = 0)
            continue;
          }
          const double point_pet =
            ego_trajectory_points_[ego_first_overlap_idx - 1].time_from_start_s -
            obstacle_trajectory_point.time_from_start_s;
          if (point_pet >= 0) {  // Only consider the case where obstacle leaves before ego arrives
                                 // （PET > 0） and occlusion occurs （PET = 0）
            obstacle_pet = std::min(obstacle_pet, point_pet);
          }
        }
      }

      if (obstacle_pet != std::numeric_limits<double>::max()) {
        Accumulator<double> obstacle_pet_metric;
        obstacle_pet_metric.add(obstacle_pet);
        obstacle_metrics_[Metric::obstacle_pet].emplace_back(object_uuid, obstacle_pet_metric);
      }
    }

    // ------------------------------------------------------------------------------------------------
    // 8. get `obstacle_drac` metrics

    if (metrics_need_[Metric::obstacle_drac]) {
      double obstacle_drac = 0.0;
      const double ego_start_vel = first_ego_point.velocity_mps;

      for (size_t i = 1; i < obstacle_trajectory_points_.size(); ++i) {
        const auto & obstacle_trajectory_point = obstacle_trajectory_points_[i];
        const auto & ego_trajectory_point = ego_trajectory_points_[i];
        if (!obstacle_trajectory_point.is_collision_with_ego_trajectory) continue;

        // calculate ego_end_vel at the collision point needed for deceleration:
        //  - ego decelerate max to stop.
        //  - consider two cases of forward and backward.
        const double yaw_diff = tf2::getYaw(ego_trajectory_point.pose.orientation) -
                                tf2::getYaw(obstacle_trajectory_point.pose.orientation);
        double ego_end_vel = obstacle_trajectory_point.velocity_mps * std::cos(yaw_diff);
        ego_end_vel =
          ego_start_vel >= 0.0 ? std::max(ego_end_vel, 0.0) : std::min(ego_end_vel, 0.0);

        // calculate DRAC
        const double distance_to_collision = ego_trajectory_point.distance_from_start_m;
        const double point_drac = (ego_end_vel * ego_end_vel - ego_start_vel * ego_start_vel) /
                                  (2.0 * distance_to_collision + 1e-6);
        obstacle_drac = std::max(obstacle_drac, std::abs(point_drac));
      }

      // Add to metric statistics if we found any valid DRAC value
      if (obstacle_drac > 0.0) {
        Accumulator<double> obstacle_drac_metric;
        obstacle_drac_metric.add(obstacle_drac);
        obstacle_metrics_[Metric::obstacle_drac].emplace_back(object_uuid, obstacle_drac_metric);
      }
    }
  }
}
void ObstacleMetricsCalculator::CollectWorstMetrics()
{
  for (const auto & [metric, is_needed] : metrics_need_) {
    if (!is_needed) {
      continue;
    }

    const auto & metric_values = obstacle_metrics_[metric];
    if (metric_values.empty()) {
      continue;
    }

    const bool find_min =
      (metric == Metric::obstacle_distance || metric == Metric::obstacle_ttc ||
       metric == Metric::obstacle_pet);

    double worst_value =
      find_min ? std::numeric_limits<double>::max() : std::numeric_limits<double>::lowest();

    for (const auto & [obj_id, accumulator] : metric_values) {
      if (accumulator.count() == 0) {
        continue;
      }

      if (find_min) {
        worst_value = std::min(worst_value, accumulator.min());
      } else {  // Metric::obstacle_drac
        worst_value = std::max(worst_value, accumulator.max());
      }
    }

    Accumulator<double> worst_accumulator;
    worst_accumulator.add(worst_value);
    if (parameters.worst_only) {
      obstacle_metrics_[metric].clear();
    }
    obstacle_metrics_[metric].emplace_back("worst", worst_accumulator);
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

  CollectWorstMetrics();
}
}  // namespace planning_diagnostics
