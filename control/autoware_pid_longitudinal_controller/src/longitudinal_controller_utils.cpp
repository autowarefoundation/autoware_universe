// Copyright 2018-2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/pid_longitudinal_controller/longitudinal_controller_utils.hpp"

#include "autoware/trajectory/utils/find_nearest.hpp"
#include "autoware/trajectory/utils/velocity.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace autoware::motion::control::pid_longitudinal_controller
{
namespace longitudinal_utils
{

double calcStopDistance(
  const Pose & current_pose, const TrajectoryExperimental & traj, const double max_dist,
  const double max_yaw)
{
  static_cast<void>(max_dist);
  static_cast<void>(max_yaw);

  const auto bases = traj.get_underlying_bases();
  if (bases.size() <= 1) {
    return 0.0;
  }

  const double ego_s =
    autoware::experimental::trajectory::find_nearest_index(traj, current_pose.position);
  const double stop_s =
    autoware::experimental::trajectory::search_zero_velocity_position(traj).value_or(traj.length());

  const double signed_length_on_traj = stop_s - ego_s;
  if (std::isnan(signed_length_on_traj)) {
    return 0.0;
  }
  return signed_length_on_traj;
}

double getPitchByPose(const Quaternion & quaternion_msg)
{
  double roll, pitch, yaw;
  tf2::Quaternion quaternion;
  tf2::fromMsg(quaternion_msg, quaternion);
  tf2::Matrix3x3{quaternion}.getRPY(roll, pitch, yaw);

  return pitch;
}

double getPitchByTraj(
  const TrajectoryExperimental & trajectory, const double start_base, const double wheel_base)
{
  const auto bases = trajectory.get_underlying_bases();
  if (bases.size() <= 1) {
    return 0.0;
  }

  const double clamped_start_base = std::clamp(start_base, 0.0, trajectory.length());
  const auto [pitch_start_base, end_base] =
    clamped_start_base + wheel_base <= trajectory.length()
      ? std::make_pair(clamped_start_base, clamped_start_base + wheel_base)
      : std::make_pair(
          std::min(clamped_start_base, bases.at(bases.size() - 2)),
          std::min(clamped_start_base + wheel_base, bases.at(bases.size() - 1)));
  const auto start_point = trajectory.compute(pitch_start_base);
  const auto end_point = trajectory.compute(end_base);
  return autoware_utils::calc_elevation_angle(start_point.pose.position, end_point.pose.position);
}

Pose calcPoseAfterTimeDelay(
  const Pose & current_pose, const double delay_time, const double current_vel,
  const double current_acc)
{
  if (delay_time <= 0.0) {
    return current_pose;
  }

  // check time to stop
  const double time_to_stop = -current_vel / current_acc;

  const double delay_time_calculation =
    time_to_stop > 0.0 && time_to_stop < delay_time ? time_to_stop : delay_time;
  // simple linear prediction
  const double yaw = tf2::getYaw(current_pose.orientation);
  const double running_distance = delay_time_calculation * current_vel + 0.5 * current_acc *
                                                                           delay_time_calculation *
                                                                           delay_time_calculation;
  const double dx = running_distance * std::cos(yaw);
  const double dy = running_distance * std::sin(yaw);

  auto pred_pose = current_pose;
  pred_pose.position.x += dx;
  pred_pose.position.y += dy;
  return pred_pose;
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val)
{
  const double diff_raw = (input_val - prev_val) / dt;
  const double diff = std::min(std::max(diff_raw, min_val), max_val);
  const double filtered_val = prev_val + diff * dt;
  return filtered_val;
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val)
{
  const double max_val = std::fabs(lim_val);
  const double min_val = -max_val;
  return applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val);
}

}  // namespace longitudinal_utils
}  // namespace autoware::motion::control::pid_longitudinal_controller
