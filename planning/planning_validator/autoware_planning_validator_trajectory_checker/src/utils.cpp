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

#include "autoware/planning_validator_trajectory_checker/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator::trajectory_checker_utils
{
using autoware_utils::calc_curvature;
using autoware_utils::calc_distance2d;
using autoware_utils::get_point;

namespace
{
void takeBigger(double & v_max, size_t & i_max, double v, size_t i)
{
  if (v_max < v) {
    v_max = v;
    i_max = i;
  }
}
void takeSmaller(double & v_min, size_t & i_min, double v, size_t i)
{
  if (v_min > v) {
    v_min = v;
    i_min = i;
  }
}
}  // namespace

std::pair<double, size_t> getAbsMaxValAndIdx(const std::vector<double> & v)
{
  const auto iter = std::max_element(
    v.begin(), v.end(), [](const auto & a, const auto & b) { return std::abs(a) < std::abs(b); });
  const auto idx = std::distance(v.begin(), iter);
  return {std::abs(*iter), idx};
}

// calculate curvature from three points with curvature_distance
void calcCurvature(const Trajectory & trajectory, std::vector<double> & curvature_vector)
{
  curvature_vector = std::vector<double>(trajectory.points.size(), 0.0);
  if (trajectory.points.size() < 3) {
    return;
  }

  // calc arc length array: arc_length(3) - arc_length(0) is distance from point(3) to point(0)
  std::vector<double> arc_length(trajectory.points.size(), 0.0);
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    arc_length.at(i) =
      arc_length.at(i - 1) + calc_distance2d(trajectory.points.at(i - 1), trajectory.points.at(i));
  }

  constexpr double curvature_distance = 1.0;  // [m]

  size_t first_distant_index = 0;
  size_t last_distant_index = trajectory.points.size() - 1;
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    // find the previous point
    size_t prev_idx = 0;
    for (size_t j = i - 1; j > 0; --j) {
      if (arc_length.at(i) - arc_length.at(j) > curvature_distance) {
        if (first_distant_index == 0) {
          first_distant_index = i;  // save first index that meets distance requirement
        }
        prev_idx = j;
        break;
      }
    }

    // find the next point
    size_t next_idx = trajectory.points.size() - 1;
    for (size_t j = i + 1; j < trajectory.points.size(); ++j) {
      if (arc_length.at(j) - arc_length.at(i) > curvature_distance) {
        last_distant_index = i;  // save last index that meets distance requirement
        next_idx = j;
        break;
      }
    }

    const auto p1 = get_point(trajectory.points.at(prev_idx));
    const auto p2 = get_point(trajectory.points.at(i));
    const auto p3 = get_point(trajectory.points.at(next_idx));
    try {
      curvature_vector.at(i) = autoware_utils::calc_curvature(p1, p2, p3);
    } catch (...) {
      curvature_vector.at(i) = 0.0;  // maybe distance is too close
    }
  }

  // use previous or last curvature where the distance is not enough
  for (size_t i = first_distant_index; i > 0; --i) {
    curvature_vector.at(i - 1) = curvature_vector.at(i);
  }
  for (size_t i = last_distant_index; i < curvature_vector.size() - 1; ++i) {
    curvature_vector.at(i + 1) = curvature_vector.at(i);
  }
}

std::pair<double, size_t> calcMaxCurvature(const Trajectory & trajectory)
{
  if (trajectory.points.size() < 3) {
    return {0.0, 0};
  }

  std::vector<double> curvature_vector;
  calcCurvature(trajectory, curvature_vector);

  const auto max_curvature_it = std::max_element(curvature_vector.begin(), curvature_vector.end());
  const size_t index = std::distance(curvature_vector.begin(), max_curvature_it);

  return {*max_curvature_it, index};
}

void calc_interval_distance(
  const Trajectory & trajectory, std::vector<double> & interval_distance_vector)
{
  interval_distance_vector.clear();

  if (trajectory.points.size() <= 1) {
    return;
  }

  interval_distance_vector.resize(trajectory.points.size() - 1, 0.0);

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    const auto d = calc_distance2d(trajectory.points.at(i), trajectory.points.at(i + 1));
    interval_distance_vector.at(i) = d;
  }
}

std::pair<double, size_t> calcMaxIntervalDistance(const Trajectory & trajectory)
{
  if (trajectory.points.size() <= 1) {
    return {0.0, 0};
  }
  std::vector<double> interval_distance_vector;
  calc_interval_distance(trajectory, interval_distance_vector);

  if (interval_distance_vector.empty()) {
    return {0.0, 0};
  }

  const auto max_interval_it =
    std::max_element(interval_distance_vector.begin(), interval_distance_vector.end());
  const size_t max_index = std::distance(interval_distance_vector.begin(), max_interval_it);

  return {*max_interval_it, max_index};
}

void calc_lateral_acceleration(
  const Trajectory & trajectory, std::vector<double> & lateral_acceleration_vector)
{
  lateral_acceleration_vector.resize(trajectory.points.size(), 0.0);

  // We need at least three points to compute curvature
  if (trajectory.points.size() < 3) {
    return;
  }

  std::vector<double> curvature_vector;
  calcCurvature(trajectory, curvature_vector);

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto v_lon = trajectory.points.at(i).longitudinal_velocity_mps;

    lateral_acceleration_vector.at(i) = v_lon * v_lon * curvature_vector.at(i);
  }
}

std::pair<double, size_t> calcMaxLateralAcceleration(const Trajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return {0.0, 0};
  }

  std::vector<double> lateral_acceleration_vector;
  calc_lateral_acceleration(trajectory, lateral_acceleration_vector);

  if (lateral_acceleration_vector.empty()) {
    return {0.0, 0};
  }

  const auto max_it = std::max_element(
    lateral_acceleration_vector.begin(), lateral_acceleration_vector.end(),
    [](double a, double b) { return std::abs(a) < std::abs(b); });
  const size_t max_index = std::distance(lateral_acceleration_vector.begin(), max_it);

  return {*max_it, max_index};
}

void calc_lateral_jerk(const Trajectory & trajectory, std::vector<double> & lateral_jerk_vector)
{
  // Handle trajectories with insufficient points
  if (trajectory.points.size() < 2) {
    lateral_jerk_vector = std::vector<double>(trajectory.points.size(), 0.0);
    return;
  }
  std::vector<double> curvature_vector;
  calcCurvature(trajectory, curvature_vector);

  // Initialize lateral jerk array with zeros
  lateral_jerk_vector = std::vector<double>(trajectory.points.size(), 0.0);

  // Calculate lateral jerk for each point
  // Note: The complete formula for lateral jerk is:
  // j_lat = v_lon^3 * (dk/ds) + 3 * v_lon^2 * a_lon * k
  // However, the dk/ds term is omitted here because the curvature calculation
  // is currently unstable, making it difficult to derive an accurate rate of
  // curvature change. Therefore, we only use the second term for a stable estimation.

  // TODO(Sugahara): When the curvature calculation becomes stable, include the v_lon^3 * (dk/ds)
  // term in the lateral jerk calculation for a more accurate result.
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const double v_lon = trajectory.points.at(i).longitudinal_velocity_mps;
    const double a_lon = trajectory.points.at(i).acceleration_mps2;

    // Calculate lateral jerk using simplified formula: jerk_lat = 3 * v_lon^2 * a_lon * curvature
    lateral_jerk_vector.at(i) = 3.0 * v_lon * v_lon * a_lon * curvature_vector.at(i);
  }
}

std::pair<double, size_t> calc_max_lateral_jerk(const Trajectory & trajectory)
{
  std::vector<double> lateral_jerk_vector;
  calc_lateral_jerk(trajectory, lateral_jerk_vector);

  if (lateral_jerk_vector.empty()) {
    return {0.0, 0};
  }

  // Find index of maximum absolute lateral jerk
  const auto max_it = std::max_element(
    lateral_jerk_vector.begin(), lateral_jerk_vector.end(),
    [](double a, double b) { return std::abs(a) < std::abs(b); });

  const size_t max_index = std::distance(lateral_jerk_vector.begin(), max_it);

  return {std::abs(*max_it), max_index};
}

std::pair<double, size_t> getMaxLongitudinalAcc(const Trajectory & trajectory)
{
  double max_acc = 0.0;
  size_t max_index = 0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    takeBigger(max_acc, max_index, trajectory.points.at(i).acceleration_mps2, i);
  }
  return {max_acc, max_index};
}

std::pair<double, size_t> getMinLongitudinalAcc(const Trajectory & trajectory)
{
  double min_acc = 0.0;
  size_t min_index = 0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    takeSmaller(min_acc, min_index, trajectory.points.at(i).acceleration_mps2, i);
  }
  return {min_acc, min_index};
}

std::pair<double, size_t> calcMaxRelativeAngles(const Trajectory & trajectory)
{
  // We need at least three points to compute relative angle
  const size_t relative_angle_points_num = 3;
  if (trajectory.points.size() < relative_angle_points_num) {
    return {0.0, 0};
  }

  double max_relative_angles = 0.0;
  size_t max_index = 0;

  for (size_t i = 0; i <= trajectory.points.size() - relative_angle_points_num; ++i) {
    const auto & p1 = trajectory.points.at(i).pose.position;
    const auto & p2 = trajectory.points.at(i + 1).pose.position;
    const auto & p3 = trajectory.points.at(i + 2).pose.position;

    const auto angle_a = autoware_utils::calc_azimuth_angle(p1, p2);
    const auto angle_b = autoware_utils::calc_azimuth_angle(p2, p3);

    // convert relative angle to [-pi ~ pi]
    const auto relative_angle = std::abs(autoware_utils::normalize_radian(angle_b - angle_a));

    takeBigger(max_relative_angles, max_index, std::abs(relative_angle), i);
  }

  return {max_relative_angles, max_index};
}

void calcSteeringAngles(
  const Trajectory & trajectory, const double wheelbase, std::vector<double> & steering_vector)
{
  const auto curvatureToSteering = [](const auto k, const auto wheelbase) {
    return std::atan(k * wheelbase);
  };

  std::vector<double> curvature_vector;
  calcCurvature(trajectory, curvature_vector);

  steering_vector.clear();
  for (const auto k : curvature_vector) {
    steering_vector.push_back(curvatureToSteering(k, wheelbase));
  }
}

std::pair<double, size_t> calcMaxSteeringAngles(
  const Trajectory & trajectory, const double wheelbase)
{
  std::vector<double> steering_vector;
  calcSteeringAngles(trajectory, wheelbase, steering_vector);

  return getAbsMaxValAndIdx(steering_vector);
}

std::pair<double, size_t> calcMaxSteeringRates(
  const Trajectory & trajectory, const double wheelbase)
{
  if (trajectory.points.size() < 1) {
    return {0.0, 0};
  }

  std::vector<double> steering_vector;
  calcSteeringAngles(trajectory, wheelbase, steering_vector);

  double max_steering_rate = 0.0;
  size_t max_index = 0;
  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    const auto & p_prev = trajectory.points.at(i);
    const auto & p_next = trajectory.points.at(i + 1);
    const auto delta_s = calc_distance2d(p_prev, p_next);
    const auto v = 0.5 * (p_next.longitudinal_velocity_mps + p_prev.longitudinal_velocity_mps);
    const auto dt = delta_s / std::max(v, 1.0e-5);

    const auto steer_prev = steering_vector.at(i);
    const auto steer_next = steering_vector.at(i + 1);

    const auto steer_rate = (steer_next - steer_prev) / dt;
    takeBigger(max_steering_rate, max_index, std::abs(steer_rate), i);
  }

  return {max_steering_rate, max_index};
}

bool checkFinite(const TrajectoryPoint & point)
{
  const auto & p = point.pose.position;
  const auto & o = point.pose.orientation;

  using std::isfinite;
  const bool p_result = isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
  const bool quat_result = isfinite(o.x) && isfinite(o.y) && isfinite(o.z) && isfinite(o.w);
  const bool v_result = isfinite(point.longitudinal_velocity_mps);
  const bool w_result = isfinite(point.heading_rate_rps);
  const bool a_result = isfinite(point.acceleration_mps2);

  return quat_result && p_result && v_result && w_result && a_result;
}

}  // namespace autoware::planning_validator::trajectory_checker_utils
