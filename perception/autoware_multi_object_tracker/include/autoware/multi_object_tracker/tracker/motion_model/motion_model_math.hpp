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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_MATH_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_MATH_HPP_

#include <Eigen/Core>

#include <geometry_msgs/msg/pose.hpp>

#include <cmath>

namespace autoware::multi_object_tracker::motion_model_math
{

constexpr double kUnobservedCov = 0.1 * 0.1;  // [m^2 or rad^2]

// Result type for rotateDiagCov2D.
struct RotatedCov2D
{
  double xx, xy, yy;
};

// Rotate diagonal process-noise (cov_long, cov_lat) by yaw into world frame.
// Caller provides precomputed sin^2, cos^2, sin*cos to avoid extra trig.
// Formula: R(yaw) * diag(cov_long, cov_lat) * R(yaw)^T
inline RotatedCov2D rotateDiagCov2D(
  const double cov_long, const double cov_lat, const double sin_yaw_sq, const double cos_yaw_sq,
  const double sin_cos_yaw)
{
  return {
    cov_long * cos_yaw_sq + cov_lat * sin_yaw_sq,  // xx
    (cov_long - cov_lat) * sin_cos_yaw,            // xy = yx
    cov_long * sin_yaw_sq + cov_lat * cos_yaw_sq   // yy
  };
}

// Rotate symmetric 2x2 covariance M by angle -yaw: R(-yaw) * M * R(-yaw)^T
// Caller provides precomputed cos_yaw/sin_yaw to avoid extra trig.
inline Eigen::Matrix2d rotateCov2D(
  const Eigen::Matrix2d & cov, const double cos_yaw, const double sin_yaw)
{
  const double cos_yaw_sq = cos_yaw * cos_yaw;
  const double sin_yaw_sq = sin_yaw * sin_yaw;
  const double sin_cos_yaw = sin_yaw * cos_yaw;
  const double a = cov(0, 0), b = cov(0, 1), d = cov(1, 1);
  Eigen::Matrix2d result;
  result(0, 0) = cos_yaw_sq * a + 2.0 * sin_cos_yaw * b + sin_yaw_sq * d;
  result(0, 1) = sin_cos_yaw * (d - a) + (cos_yaw_sq - sin_yaw_sq) * b;
  result(1, 0) = result(0, 1);
  result(1, 1) = sin_yaw_sq * a - 2.0 * sin_cos_yaw * b + cos_yaw_sq * d;
  return result;
}

inline double clampSymmetric(const double val, const double abs_max)
{
  if (val > abs_max) return abs_max;
  if (val < -abs_max) return -abs_max;
  return val;
}

inline void setOrientationFromYaw(geometry_msgs::msg::Pose & pose, const double yaw)
{
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
}

// Adjust measured_yaw to be continuous with estimated_yaw across pi-wrap boundaries.
inline double fixYawContinuity(const double estimated_yaw, const double measured_yaw)
{
  return measured_yaw + M_PI * std::round((estimated_yaw - measured_yaw) / M_PI);
}

}  // namespace autoware::multi_object_tracker::motion_model_math

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_MATH_HPP_
