// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR__UTILS_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

std::pair<double, size_t> getAbsMaxValAndIdx(const std::vector<double> & v);

Trajectory resampleTrajectory(const Trajectory & trajectory, const double min_interval);

Trajectory getStopTrajectory(
  const Trajectory & trajectory, const int nearest_traj_idx, const double current_vel,
  const double decel);

void calcCurvature(
  const Trajectory & trajectory, std::vector<double> & curvatures,
  const double curvature_distance = 1.0);

void calcSteeringAngles(
  const Trajectory & trajectory, const double wheelbase, std::vector<double> & steering_array);

std::pair<double, size_t> calcMaxCurvature(const Trajectory & trajectory);

void calc_interval_distance(
  const Trajectory & trajectory, std::vector<double> & interval_distance_arr);

std::pair<double, size_t> calcMaxIntervalDistance(const Trajectory & trajectory);

void calc_lateral_acceleration(
  const Trajectory & trajectory, std::vector<double> & lateral_acceleration_arr);

std::pair<double, size_t> calcMaxLateralAcceleration(const Trajectory & trajectory);

void calc_interval_time(const Trajectory & trajectory, std::vector<double> & time_interval_arr);

void calc_lateral_jerk(const Trajectory & trajectory, std::vector<double> & lateral_jerk_arr);

std::pair<double, size_t> calc_max_lateral_jerk(const Trajectory & trajectory);

std::pair<double, size_t> getMaxLongitudinalAcc(const Trajectory & trajectory);

std::pair<double, size_t> getMinLongitudinalAcc(const Trajectory & trajectory);

std::pair<double, size_t> calcMaxRelativeAngles(const Trajectory & trajectory);

std::pair<double, size_t> calcMaxSteeringAngles(
  const Trajectory & trajectory, const double wheelbase);

std::pair<double, size_t> calcMaxSteeringRates(
  const Trajectory & trajectory, const double wheelbase);

std::optional<std::pair<std::vector<TrajectoryPoint>, std::vector<Box>>> check_collision(
  const PredictedObjects & predicted_objects, const Trajectory & trajectory,
  const geometry_msgs::msg::Point & current_ego_position, const VehicleInfo & vehicle_info,
  const double trajectory_to_object_distance_threshold,
  const double ego_to_object_distance_threshold, const double time_tolerance_threshold);

Rtree make_ego_footprint_rtree(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const VehicleInfo & vehicle_info);

std::optional<PredictedObjects> filter_objects(
  const PredictedObjects & objects,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double trajectory_to_object_distance_threshold,
  const double ego_to_object_distance_threshold);

std::optional<PredictedPath> find_highest_confidence_path(const PredictedObject & object);

void make_predicted_object_rtree(
  const PredictedPath & highest_confidence_path, const Shape & object_shape,
  const double predicted_time_step, std::vector<BoxTimeIndexPair> & predicted_object_rtree_nodes);

std::vector<std::pair<size_t, Box>> detect_collisions(
  const Rtree & ego_rtree, const Rtree & predicted_object_rtree, double time_tolerance);

bool checkFinite(const TrajectoryPoint & point);

void shiftPose(geometry_msgs::msg::Pose & pose, double longitudinal);

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__UTILS_HPP_
