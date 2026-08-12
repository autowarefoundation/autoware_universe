// Copyright 2022 TIER IV, Inc.
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

#include "arrival_checker.hpp"

#include <autoware_utils/math/normalization.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <deque>

namespace autoware::mission_planner_universe
{

namespace
{
bool is_vehicle_stopped(
  const std::deque<geometry_msgs::msg::TwistStamped> & twist_buffer, const double duration)
{
  if (twist_buffer.empty()) {
    return false;
  }

  constexpr double squared_stop_velocity = 1e-3 * 1e-3;

  const rclcpp::Time now(twist_buffer.front().header.stamp);
  const auto time_buffer_back = now - rclcpp::Time(twist_buffer.back().header.stamp);
  if (time_buffer_back.seconds() < duration) {
    return false;
  }

  // Get velocities within the stop duration.
  for (const auto & velocity : twist_buffer) {
    const double x = velocity.twist.linear.x;
    const double y = velocity.twist.linear.y;
    const double z = velocity.twist.linear.z;
    const double squared_velocity = (x * x) + (y * y) + (z * z);
    if (squared_stop_velocity <= squared_velocity) {
      return false;
    }

    const auto time_diff = now - rclcpp::Time(velocity.header.stamp);
    if (time_diff.seconds() >= duration) {
      break;
    }
  }

  return true;
}
}  // namespace

ArrivalChecker::ArrivalChecker(const ArrivalCheckerThreshold & threshold) : threshold_(threshold)
{
}

void ArrivalChecker::clear_goal()
{
  // Ignore the modified goal after the route is cleared.
  goal_with_uuid_ = std::nullopt;
}

void ArrivalChecker::set_goal(const PoseWithUuidStamped & goal)
{
  // Ignore the modified goal for the previous route using uuid.
  goal_with_uuid_ = goal;
}

void ArrivalChecker::update(const Odometry & odometry)
{
  odometry_ = odometry;

  TwistStamped twist;
  twist.header = odometry.header;
  twist.twist = odometry.twist.twist;
  twist_buffer_.push_front(twist);

  const rclcpp::Time now(odometry.header.stamp);
  while (!twist_buffer_.empty()) {
    // Check oldest data time.
    const auto time_diff = now - rclcpp::Time(twist_buffer_.back().header.stamp);

    // Finish when oldest data is newer than threshold.
    if (time_diff.seconds() <= velocity_buffer_time_sec) {
      break;
    }

    // Remove old data.
    twist_buffer_.pop_back();
  }
}

bool ArrivalChecker::is_arrived() const
{
  if (!odometry_ || !goal_with_uuid_) {
    return false;
  }
  const auto & goal = goal_with_uuid_.value();
  const auto & odometry = odometry_.value();

  // Check frame id
  if (goal.header.frame_id != odometry.header.frame_id) {
    return false;
  }

  // Compute position difference in goal frame
  const double dx = odometry.pose.pose.position.x - goal.pose.position.x;
  const double dy = odometry.pose.pose.position.y - goal.pose.position.y;

  const double yaw_goal = tf2::getYaw(goal.pose.orientation);
  const double longitudinal_offset_to_goal = std::cos(yaw_goal) * dx + std::sin(yaw_goal) * dy;
  const double lateral_offset_to_goal = -std::sin(yaw_goal) * dx + std::cos(yaw_goal) * dy;

  const double yaw_pose = tf2::getYaw(odometry.pose.pose.orientation);
  const double yaw_diff = autoware_utils::normalize_radian(yaw_pose - yaw_goal);

  // Check lateral distance.
  const bool is_within_lateral_range =
    std::abs(lateral_offset_to_goal) <= threshold_.lateral_distance;

  // Check longitudinal distance.
  // The acceptable range is [-longitudinal_undershoot_distance, +longitudinal_overshoot_distance].
  const bool is_within_longitudinal_range =
    longitudinal_offset_to_goal >= -threshold_.longitudinal_undershoot_distance &&
    longitudinal_offset_to_goal <= threshold_.longitudinal_overshoot_distance;

  // Check angle.
  const bool is_within_angle_range = std::fabs(yaw_diff) <= threshold_.angle;

  if (!is_within_lateral_range || !is_within_longitudinal_range || !is_within_angle_range) {
    return false;
  }

  // Check vehicle stopped.
  return is_vehicle_stopped(twist_buffer_, threshold_.duration);
}

}  // namespace autoware::mission_planner_universe
