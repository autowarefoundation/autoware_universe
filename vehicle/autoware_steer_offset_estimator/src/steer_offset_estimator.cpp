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

#include "autoware/steer_offset_estimator/steer_offset_estimator.hpp"

#include "autoware/steer_offset_estimator/utils.hpp"

#include <tl_expected/expected.hpp>

#include <cmath>
#include <vector>
namespace autoware::steer_offset_estimator
{

SteerOffsetEstimator::SteerOffsetEstimator(const SteerOffsetEstimatorParameters & parameters)
: params_(parameters),
  estimated_offset_(parameters.initial_offset),
  covariance_(parameters.initial_covariance)
{
}

tl::expected<SteerOffsetEstimationUpdated, SteerOffsetEstimationNotUpdated>
SteerOffsetEstimator::update(
  const std::vector<PoseStamped> & poses, const std::vector<SteeringReport> & steers)
{
  if (poses.empty()) {
    return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"poses is empty"});
  }

  if (steers.empty()) {
    return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"steers is empty"});
  }

  geometry_msgs::msg::Twist twist;

  // Determine which poses to use for motion calculation
  if (!previous_pose_) {
    // First call - need at least 2 poses to calculate motion
    if (poses.size() < 2) {
      previous_pose_ = poses.back();
      return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"previous_pose has not been set"});
    }
    // Use first pose as previous, last pose as current
    twist = utils::calc_twist_from_pose(poses[0], poses.back());
    previous_pose_ = poses.back();
  } else {
    // Subsequent calls - use stored previous pose and current pose
    twist = utils::calc_twist_from_pose(previous_pose_.value(), poses.back());
    previous_pose_ = poses.back();
  }

  double velocity = twist.linear.x;
  double angular_velocity = twist.angular.z;
  double steering_angle = utils::calc_average_steer(steers);

  // Validate input data quality
  if (velocity < params_.min_velocity) {
    return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"velocity is too low"});
  }
  if (std::abs(steering_angle) > params_.max_steer) {
    return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"steering angle is too large"});
  }

  const double phi = velocity / params_.wheel_base;

  const double y = angular_velocity - phi * steering_angle;

  double denominator =
    params_.forgetting_factor * params_.measurement_noise + phi * phi * covariance_;

  const double kalman_gain = (covariance_ * phi) / denominator;

  if (denominator < params_.denominator_floor) {
    denominator = params_.denominator_floor;
  }

  const double residual = y - phi * estimated_offset_;

  estimated_offset_ = estimated_offset_ + kalman_gain * residual;

  covariance_ = (covariance_ - (covariance_ * phi * phi * covariance_) / denominator) /
                params_.forgetting_factor;

  if (covariance_ < params_.covariance_floor) {
    covariance_ = params_.covariance_floor;
  }

  SteerOffsetEstimationUpdated updated_result;
  updated_result.offset = estimated_offset_;
  updated_result.covariance = covariance_;
  updated_result.velocity = velocity;
  updated_result.angular_velocity = angular_velocity;
  updated_result.steering_angle = steering_angle;
  updated_result.kalman_gain = kalman_gain;
  updated_result.residual = residual;
  return updated_result;
}

}  // namespace autoware::steer_offset_estimator
