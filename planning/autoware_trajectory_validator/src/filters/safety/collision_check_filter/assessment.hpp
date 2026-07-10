// Copyright 2026 TIER IV, Inc.
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

#ifndef FILTERS__SAFETY__COLLISION_CHECK_FILTER__ASSESSMENT_HPP_
#define FILTERS__SAFETY__COLLISION_CHECK_FILTER__ASSESSMENT_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"
#include "parameter.hpp"
#include "trajectory_utils.hpp"
#include "types.hpp"

#include <Eigen/Geometry>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/pose_deviation.hpp>

#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment
{
std::vector<TrajectoryData> generate_object_trajectories(
  const FilterContext & context, double required_time_horizon, double object_assumed_acceleration,
  double time_resolution, const DracParamMap & drac_param_map);

DracArtifact assess(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache, const FilterContext & context,
  const DracParamMap & drac_param_map, const GlobalParams & global_params,
  const VehicleInfo & vehicle_info);
}  // namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment

namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
{
template <typename PosePoints, typename Object>
double compute_longitudinal_velocity(const PosePoints & points, const Object & object)
{
  if (points.empty()) {
    throw std::invalid_argument("points must not be empty");
  }

  constexpr double min_path_end_to_end_distance = 1e-3;

  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const bool use_path_yaw =
    points.size() >= 2 && autoware_utils_geometry::calc_distance2d(points.front(), points.back()) >=
                            min_path_end_to_end_distance;
  const double object_yaw_relative_to_points =
    use_path_yaw ? autoware::motion_utils::calcYawDeviation(points, object_pose, true)
                 : autoware::universe_utils::calcYawDeviation(points.front(), object_pose);
  const Eigen::Rotation2Dd object_to_points_rotation(object_yaw_relative_to_points);

  const auto & object_twist = object.kinematics.initial_twist_with_covariance.twist;
  const Eigen::Vector2d object_velocity_in_object_frame(
    object_twist.linear.x, object_twist.linear.y);
  const Eigen::Vector2d object_velocity_in_points_frame =
    object_to_points_rotation * object_velocity_in_object_frame;

  return object_velocity_in_points_frame.x();
}

std::optional<double> compute_distance_to_collision(
  const TrajectoryData & ego_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & object);

RssArtifact assess(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache, const FilterContext & context,
  const RssParamMap & rss_param_map, const GlobalParams & global_params,
  const VehicleInfo & vehicle_info);
}  // namespace autoware::trajectory_validator::plugin::safety::rss_deceleration

#endif  // FILTERS__SAFETY__COLLISION_CHECK_FILTER__ASSESSMENT_HPP_
