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

#include "autoware/boundary_departure_checker/footprint_generator/steering_footprint.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <tf2/utils.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <optional>
#include <vector>

namespace autoware::boundary_departure_checker
{

namespace
{
geometry_msgs::msg::Pose update_pose_with_bicycle_model(
  const geometry_msgs::msg::Pose & pose, const double steering_angle, const double velocity,
  const double dt, const double wheel_base)
{
  const auto old_heading = tf2::getYaw(pose.orientation);
  const double rotation_rate = velocity * std::tan(steering_angle) / wheel_base;

  constexpr double epsilon = 1e-8;
  auto updated_pose = pose;
  if (std::abs(rotation_rate) < epsilon) {  // Case 1: straight line
    const double distance_traveled = velocity * dt;
    updated_pose.position.x += distance_traveled * std::cos(old_heading);
    updated_pose.position.y += distance_traveled * std::sin(old_heading);
  } else {  // Case 2: arc
    const double turning_radius = velocity / rotation_rate;
    const double new_heading = old_heading + rotation_rate * dt;
    updated_pose.position.x += turning_radius * (std::sin(new_heading) - std::sin(old_heading));
    updated_pose.position.y += turning_radius * (std::cos(old_heading) - std::cos(new_heading));
    updated_pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(new_heading);
  }
  return updated_pose;
}
}  // anonymous namespace

Footprints SteeringFootprintGenerator::generate(
  const TrajectoryPoints & pred_traj, const vehicle_info_utils::VehicleInfo & info,
  const Param & param, [[maybe_unused]] const FootprintMargin & uncertainty_fp_margin)
{
  const auto config_opt = param.get_abnormality_config<SteeringConfig>(type_);
  if (!config_opt || pred_traj.empty()) {
    return {};
  }
  const auto & config = config_opt->get();

  std::vector<autoware_utils_geometry::LinearRing2d> vehicle_footprints;
  vehicle_footprints.reserve(pred_traj.size());
  const auto local_vehicle_footprint = info.createFootprint(0.0, 0.0, 0.0, 0.0, 0.0, true);

  vehicle_footprints.push_back(
    autoware_utils_geometry::transform_vector(
      local_vehicle_footprint, autoware_utils_geometry::pose2transform(pred_traj.front().pose)));

  if (pred_traj.size() < 2) return vehicle_footprints;

  std::vector<double> original_steering_changes;
  original_steering_changes.reserve(pred_traj.size());
  auto t = 0.0;
  std::optional<size_t>
    delayed_index;  // the delay assumes a constant time interval along the pred_traj
  for (auto i = 0UL; i + 1 < pred_traj.size(); ++i) {
    original_steering_changes.push_back(
      pred_traj[i + 1].front_wheel_angle_rad - pred_traj[i].front_wheel_angle_rad);
    if (t >= config.delay_s) {
      if (!delayed_index) delayed_index = i;
      continue;
    }
    vehicle_footprints.push_back(
      autoware_utils_geometry::transform_vector(
        local_vehicle_footprint, autoware_utils_geometry::pose2transform(pred_traj[i].pose)));
    const auto dt = rclcpp::Duration(pred_traj[i + 1].time_from_start) -
                    rclcpp::Duration(pred_traj[i].time_from_start);
    t += dt.seconds();
  }
  if (!delayed_index) {
    delayed_index = 0UL;
  }
  auto pose = pred_traj[*delayed_index].pose;
  auto steering_angle = static_cast<double>(pred_traj[*delayed_index].front_wheel_angle_rad);
  for (auto i = *delayed_index; i + 1 < pred_traj.size(); ++i) {
    const auto prev_p = pred_traj[i];
    const auto curr_p = pred_traj[i + 1];
    const auto dt =
      rclcpp::Duration(curr_p.time_from_start) - rclcpp::Duration(prev_p.time_from_start);
    const auto v = (prev_p.longitudinal_velocity_mps + curr_p.longitudinal_velocity_mps) * 0.5;
    // simulate the ego vehicle motion
    pose = update_pose_with_bicycle_model(pose, steering_angle, v, dt.seconds(), info.wheel_base_m);
    vehicle_footprints.push_back(
      autoware_utils_geometry::transform_vector(
        local_vehicle_footprint, autoware_utils_geometry::pose2transform(pose)));

    // update the simulated steering angle
    const auto steering_change = original_steering_changes[i - *delayed_index] * config.factor +
                                 config.offset_rps * dt.seconds();
    const auto steering_rate_limit = autoware::interpolation::lerp(
      config.steering_rate_velocities_mps, config.steering_rate_limits_rps, v);
    const auto max_steering_change = steering_rate_limit * dt.seconds();
    steering_angle += std::clamp(steering_change, -max_steering_change, max_steering_change);
    steering_angle =
      std::clamp(steering_angle, -info.max_steer_angle_rad, info.max_steer_angle_rad);
  }
  return vehicle_footprints;
}

}  // namespace autoware::boundary_departure_checker
