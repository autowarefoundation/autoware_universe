// Copyright 2023-2024 Tier IV, Inc.
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

#include "autoware_frenet_planner/frenet_planner.hpp"

#include <autoware_auto_common/helper_functions/angle_utils.hpp>
#include <autoware_frenet_planner/conversions.hpp>
#include <autoware_frenet_planner/polynomials.hpp>
#include <autoware_frenet_planner/structures.hpp>
#include <autoware_sampler_common/structures.hpp>
#include <autoware_sampler_common/transform/spline_transform.hpp>
#include <eigen3/Eigen/Eigen>
#include <tf2/utils.hpp>

#include "autoware_planning_msgs/msg/path.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace autoware::frenet_planner
{
std::vector<Trajectory> generateTrajectories(
  const autoware::sampler_common::transform::Spline2D & reference_spline,
  const FrenetState & initial_state, const SamplingParameters & sampling_parameters)
{
  std::vector<Trajectory> trajectories;
  trajectories.reserve(sampling_parameters.parameters.size());
  for (const auto & parameter : sampling_parameters.parameters) {
    auto trajectory = generateCandidate(
      initial_state, parameter.target_state, parameter.target_duration,
      sampling_parameters.resolution);
    trajectory.sampling_parameter = parameter;
    calculateCartesian(reference_spline, trajectory);
    std::stringstream ss;
    ss << parameter;
    trajectory.tag = ss.str();
    trajectories.push_back(trajectory);
  }
  return trajectories;
}

std::vector<Path> generatePaths(
  const autoware::sampler_common::transform::Spline2D & reference_spline,
  const FrenetState & initial_state, const SamplingParameters & sampling_parameters)
{
  std::vector<Path> candidates;
  candidates.reserve(sampling_parameters.parameters.size());
  for (const auto parameter : sampling_parameters.parameters) {
    auto candidate =
      generateCandidate(initial_state, parameter.target_state, sampling_parameters.resolution);
    calculateCartesian(reference_spline, candidate);
    candidates.push_back(candidate);
  }
  return candidates;
}

Trajectory generateCandidate(
  const FrenetState & initial_state, const FrenetState & target_state, const double duration,
  const double time_resolution)
{
  Trajectory trajectory;
  trajectory.longitudinal_polynomial = Polynomial(
    initial_state.position.s, initial_state.longitudinal_velocity,
    initial_state.longitudinal_acceleration, target_state.position.s,
    target_state.longitudinal_velocity, target_state.longitudinal_acceleration, duration);
  trajectory.lateral_polynomial = Polynomial(
    initial_state.position.d, initial_state.lateral_velocity, initial_state.lateral_acceleration,
    target_state.position.d, target_state.lateral_velocity, target_state.lateral_acceleration,
    duration);
  for (double t = 0.0; t <= duration; t += time_resolution) {
    trajectory.times.push_back(t);
    trajectory.frenet_points.emplace_back(
      trajectory.longitudinal_polynomial->position(t), trajectory.lateral_polynomial->position(t));
  }
  return trajectory;
}

Path generateCandidate(
  const FrenetState & initial_state, const FrenetState & target_state, const double s_resolution)
{
  const auto delta_s = target_state.position.s - initial_state.position.s;
  Path path;
  path.lateral_polynomial = Polynomial(
    initial_state.position.d, initial_state.lateral_velocity, initial_state.lateral_acceleration,
    target_state.position.d, target_state.lateral_velocity, target_state.lateral_acceleration,
    delta_s);
  for (double s = s_resolution; s <= delta_s; s += s_resolution) {
    path.frenet_points.emplace_back(
      initial_state.position.s + s, path.lateral_polynomial->position(s));
  }
  return path;
}

void calculateCartesian(
  const autoware::sampler_common::transform::Spline2D & reference, Path & path)
{
  if (path.frenet_points.empty()) return;

  const auto n = path.frenet_points.size();
  path.points.reserve(n);
  path.yaws.reserve(n);
  path.lengths.reserve(n);
  path.curvatures.reserve(n);
  path.poses.reserve(n);

  // Calculate cartesian positions
  for (const auto & fp : path.frenet_points) {
    path.points.push_back(reference.cartesian(fp));
  }

  // Calculate arc lengths from cartesian points
  path.lengths.push_back(0.0);
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto dx = path.points[i + 1].x() - path.points[i].x();
    const auto dy = path.points[i + 1].y() - path.points[i].y();
    path.lengths.push_back(path.lengths.back() + std::hypot(dx, dy));
  }

  if (path.lateral_polynomial.has_value()) {
    // Werling 2010 Appendix I: analytical yaw and curvature from lateral polynomial d(s)
    const double s0 = path.frenet_points.front().s;
    for (size_t i = 0; i < n; ++i) {
      const auto & fp = path.frenet_points[i];
      const double s_local = fp.s - s0;

      // Reference path properties
      const double theta_r = reference.yaw(fp.s);
      const double kappa_r = reference.curvature(fp.s);
      const double kappa_r_deriv = reference.curvatureDerivative(fp.s);

      // Lateral polynomial derivatives
      const double d_val = path.lateral_polynomial->position(s_local);
      const double d_prim = path.lateral_polynomial->velocity(s_local);
      const double d_bis = path.lateral_polynomial->acceleration(s_local);

      // Heading: θ = θ_r + arctan(d' / (1 - κ_r · d))
      const double one_minus_krd = std::max(std::abs(1.0 - kappa_r * d_val), 1e-6) *
                                   ((1.0 - kappa_r * d_val) >= 0.0 ? 1.0 : -1.0);
      const double delta_theta = std::atan2(d_prim, one_minus_krd);
      const double theta = theta_r + delta_theta;

      // Curvature (Werling 2010 eq. from Appendix I)
      const double cos_dtheta = std::cos(delta_theta);
      const double tan_dtheta = std::tan(delta_theta);
      const double kappa =
        ((d_bis + (kappa_r_deriv * d_val + kappa_r * d_prim) * tan_dtheta) * cos_dtheta *
           cos_dtheta / one_minus_krd +
         kappa_r) *
        cos_dtheta / one_minus_krd;

      path.yaws.push_back(theta);
      path.curvatures.push_back(kappa);

      geometry_msgs::msg::Pose pose;
      pose.position.x = path.points[i].x();
      pose.position.y = path.points[i].y();
      pose.position.z = 0.0;
      pose.orientation = autoware_utils::create_quaternion_from_rpy(0.0, 0.0, theta);
      path.poses.push_back(pose);
    }
  } else {
    // Fallback: finite-difference yaw and curvature (for Bezier or reused paths)
    for (size_t i = 0; i + 1 < n; ++i) {
      const auto dx = path.points[i + 1].x() - path.points[i].x();
      const auto dy = path.points[i + 1].y() - path.points[i].y();
      const auto yaw = std::atan2(dy, dx);
      path.yaws.push_back(yaw);

      geometry_msgs::msg::Pose pose;
      pose.position.x = path.points[i].x();
      pose.position.y = path.points[i].y();
      pose.position.z = 0.0;
      pose.orientation = autoware_utils::create_quaternion_from_rpy(0.0, 0.0, yaw);
      path.poses.push_back(pose);
    }
    path.yaws.push_back(path.yaws.back());
    path.poses.push_back(path.poses.back());

    for (size_t i = 1; i < path.yaws.size(); ++i) {
      const auto dyaw =
        autoware::common::helper_functions::wrap_angle(path.yaws[i] - path.yaws[i - 1]);
      const auto dl = path.lengths[i] - path.lengths[i - 1];
      path.curvatures.push_back(dl > 1e-10 ? dyaw / dl : 0.0);
    }
    path.curvatures.push_back(path.curvatures.back());
  }
}
void calculateCartesian(
  const autoware::sampler_common::transform::Spline2D & reference, Trajectory & trajectory)
{
  if (trajectory.frenet_points.empty()) return;

  const auto n = trajectory.frenet_points.size();
  trajectory.points.reserve(n);

  // Calculate cartesian positions
  for (const auto & fp : trajectory.frenet_points)
    trajectory.points.push_back(reference.cartesian(fp));

  // Calculate arc lengths
  trajectory.lengths.push_back(0.0);
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto dx = trajectory.points[i + 1].x() - trajectory.points[i].x();
    const auto dy = trajectory.points[i + 1].y() - trajectory.points[i].y();
    trajectory.lengths.push_back(trajectory.lengths.back() + std::hypot(dx, dy));
  }

  const bool has_polynomials =
    trajectory.lateral_polynomial.has_value() && trajectory.longitudinal_polynomial.has_value();

  if (has_polynomials) {
    // Werling 2010 Appendix I: analytical yaw and curvature
    // Trajectory uses d(t) and s(t), so we convert derivatives to d-w.r.t.-s via chain rule:
    //   d'(s) = (dd/dt) / (ds/dt)
    //   d''(s) = ((d²d/dt²)(ds/dt) - (dd/dt)(d²s/dt²)) / (ds/dt)³
    trajectory.yaws.reserve(n);
    trajectory.curvatures.reserve(n);

    for (size_t i = 0; i < n; ++i) {
      const auto & fp = trajectory.frenet_points[i];
      const double time = trajectory.times[i];

      // Reference path properties at this s
      const double theta_r = reference.yaw(fp.s);
      const double kappa_r = reference.curvature(fp.s);
      const double kappa_r_deriv = reference.curvatureDerivative(fp.s);

      // Time-domain polynomial derivatives
      const double d_val = fp.d;
      const double dd_dt = trajectory.lateral_polynomial->velocity(time);
      const double d2d_dt2 = trajectory.lateral_polynomial->acceleration(time);
      const double ds_dt = trajectory.longitudinal_polynomial->velocity(time);
      const double d2s_dt2 = trajectory.longitudinal_polynomial->acceleration(time);

      // Convert to s-domain derivatives via chain rule
      const double ds_dt_safe = std::abs(ds_dt) > 1e-6 ? ds_dt : (ds_dt >= 0 ? 1e-6 : -1e-6);
      const double d_prim = dd_dt / ds_dt_safe;
      const double d_bis =
        (d2d_dt2 * ds_dt_safe - dd_dt * d2s_dt2) / (ds_dt_safe * ds_dt_safe * ds_dt_safe);

      // Heading: θ = θ_r + arctan(d' / (1 - κ_r · d))
      const double one_minus_krd_raw = 1.0 - kappa_r * d_val;
      const double one_minus_krd =
        std::max(std::abs(one_minus_krd_raw), 1e-6) * (one_minus_krd_raw >= 0.0 ? 1.0 : -1.0);
      const double delta_theta = std::atan2(d_prim, one_minus_krd);
      const double theta = theta_r + delta_theta;

      // Curvature (Werling 2010 Appendix I)
      const double cos_dtheta = std::cos(delta_theta);
      const double tan_dtheta = std::tan(delta_theta);
      const double kappa =
        ((d_bis + (kappa_r_deriv * d_val + kappa_r * d_prim) * tan_dtheta) * cos_dtheta *
           cos_dtheta / one_minus_krd +
         kappa_r) *
        cos_dtheta / one_minus_krd;

      trajectory.yaws.push_back(theta);
      trajectory.curvatures.push_back(kappa);
    }
  } else {
    // Fallback: finite-difference yaw and curvature
    for (size_t i = 0; i + 1 < n; ++i) {
      const auto dx = trajectory.points[i + 1].x() - trajectory.points[i].x();
      const auto dy = trajectory.points[i + 1].y() - trajectory.points[i].y();
      trajectory.yaws.push_back(std::atan2(dy, dx));
    }
    if (!trajectory.yaws.empty()) trajectory.yaws.push_back(trajectory.yaws.back());

    for (size_t i = 1; i < trajectory.yaws.size(); ++i) {
      const auto dyaw =
        autoware::common::helper_functions::wrap_angle(trajectory.yaws[i] - trajectory.yaws[i - 1]);
      const auto dl = trajectory.lengths[i] - trajectory.lengths[i - 1];
      trajectory.curvatures.push_back(dl > 1e-10 ? dyaw / dl : 0.0);
    }
    const auto last_curvature =
      trajectory.curvatures.empty() ? 0.0 : trajectory.curvatures.back();
    trajectory.curvatures.push_back(last_curvature);
  }

  // Calculate velocities, accelerations, jerk
  if (has_polynomials) {
    for (size_t i = 0; i < trajectory.times.size(); ++i) {
      const auto time = trajectory.times[i];
      const auto s_vel = trajectory.longitudinal_polynomial->velocity(time);
      const auto s_acc = trajectory.longitudinal_polynomial->acceleration(time);
      const auto d_vel = trajectory.lateral_polynomial->velocity(time);
      const auto d_acc = trajectory.lateral_polynomial->acceleration(time);
      // Use the analytical delta_theta for rotation instead of finite-difference d_yaws
      const double theta_r = reference.yaw(trajectory.frenet_points[i].s);
      const double delta_theta = trajectory.yaws[i] - theta_r;
      Eigen::Rotation2D rotation(delta_theta);
      Eigen::Vector2d vel_vector{s_vel, d_vel};
      Eigen::Vector2d acc_vector{s_acc, d_acc};
      const auto vel = rotation * vel_vector;
      const auto acc = rotation * acc_vector;
      trajectory.longitudinal_velocities.push_back(vel.x());
      trajectory.lateral_velocities.push_back(vel.y());
      trajectory.longitudinal_accelerations.push_back(acc.x());
      trajectory.lateral_accelerations.push_back(acc.y());
      trajectory.jerks.push_back(
        trajectory.longitudinal_polynomial->jerk(time) +
        trajectory.lateral_polynomial->jerk(time));
    }
  }
  if (trajectory.longitudinal_accelerations.empty()) {
    trajectory.longitudinal_accelerations.push_back(0.0);
    trajectory.lateral_accelerations.push_back(0.0);
  }

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = trajectory.points[i].x();
    pose.position.y = trajectory.points[i].y();
    pose.position.z = 0.0;
    pose.orientation = autoware_utils::create_quaternion_from_rpy(0.0, 0.0, trajectory.yaws[i]);
    trajectory.poses.push_back(pose);
  }
}

}  // namespace autoware::frenet_planner
