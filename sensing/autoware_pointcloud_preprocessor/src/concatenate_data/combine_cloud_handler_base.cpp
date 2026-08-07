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

#include "autoware/pointcloud_preprocessor/concatenate_data/combine_cloud_handler_base.hpp"

#include "conversion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace autoware::pointcloud_preprocessor
{

void CombineCloudHandlerBase::set_transform(
  const geometry_msgs::msg::TransformStamped & sensor_to_output_frame)
{
  sensor_to_output_transforms_[sensor_to_output_frame.child_frame_id] =
    utils::to_eigen_matrix(sensor_to_output_frame.transform);
}

std::optional<Eigen::Matrix4f> CombineCloudHandlerBase::get_transform_to_output_frame(
  const std::string & frame_id) const
{
  if (frame_id == output_frame_) return Eigen::Matrix4f::Identity();

  auto it = sensor_to_output_transforms_.find(frame_id);
  if (it == sensor_to_output_transforms_.end()) return std::nullopt;
  return it->second;
}

void CombineCloudHandlerBase::process_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & twist_msg)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header = twist_msg->header;
  msg.twist = twist_msg->twist.twist;

  // If time jumps backwards (e.g. when a rosbag restarts), clear buffer
  if (!twist_queue_.empty()) {
    if (
      utils::to_nanoseconds(twist_queue_.front().header.stamp) >
      utils::to_nanoseconds(msg.header.stamp)) {
      twist_queue_.clear();
    }
  }

  // Twist data in the queue that is older than the current twist by 1 second will be cleared.
  const int64_t cutoff_nanoseconds = utils::to_nanoseconds(msg.header.stamp) - 1'000'000'000LL;

  while (!twist_queue_.empty()) {
    if (utils::to_nanoseconds(twist_queue_.front().header.stamp) > cutoff_nanoseconds) {
      break;
    }
    twist_queue_.pop_front();
  }

  twist_queue_.push_back(msg);
}

void CombineCloudHandlerBase::process_odometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header = odometry_msg->header;
  msg.twist = odometry_msg->twist.twist;

  // If time jumps backwards (e.g. when a rosbag restarts), clear buffer
  if (!twist_queue_.empty()) {
    if (
      utils::to_nanoseconds(twist_queue_.front().header.stamp) >
      utils::to_nanoseconds(msg.header.stamp)) {
      twist_queue_.clear();
    }
  }

  // Twist data in the queue that is older than the current twist by 1 second will be cleared.
  const int64_t cutoff_nanoseconds = utils::to_nanoseconds(msg.header.stamp) - 1'000'000'000LL;

  while (!twist_queue_.empty()) {
    if (utils::to_nanoseconds(twist_queue_.front().header.stamp) > cutoff_nanoseconds) {
      break;
    }
    twist_queue_.pop_front();
  }

  twist_queue_.push_back(msg);
}

std::deque<geometry_msgs::msg::TwistStamped> CombineCloudHandlerBase::get_twist_queue()
{
  return twist_queue_;
}

Eigen::Matrix4f CombineCloudHandlerBase::compute_transform_to_adjust_for_old_timestamp(
  const builtin_interfaces::msg::Time & old_stamp, const builtin_interfaces::msg::Time & new_stamp,
  MotionCompensationStatus * status)
{
  // return identity if no twist is available
  if (twist_queue_.empty()) {
    if (status != nullptr) status->no_twist_available = true;
    return Eigen::Matrix4f::Identity();
  }

  const int64_t old_nanoseconds = utils::to_nanoseconds(old_stamp);
  const int64_t new_nanoseconds = utils::to_nanoseconds(new_stamp);

  auto old_twist_it = std::lower_bound(
    std::begin(twist_queue_), std::end(twist_queue_), old_nanoseconds,
    [](const geometry_msgs::msg::TwistStamped & x, const int64_t t) {
      return utils::to_nanoseconds(x.header.stamp) < t;
    });
  old_twist_it = old_twist_it == twist_queue_.end() ? (twist_queue_.end() - 1) : old_twist_it;

  auto new_twist_it = std::lower_bound(
    std::begin(twist_queue_), std::end(twist_queue_), new_nanoseconds,
    [](const geometry_msgs::msg::TwistStamped & x, const int64_t t) {
      return utils::to_nanoseconds(x.header.stamp) < t;
    });
  new_twist_it = new_twist_it == twist_queue_.end() ? (twist_queue_.end() - 1) : new_twist_it;

  // Accumulate the time delta in integer nanoseconds and only convert each step to seconds, so dt
  // keeps full nanosecond precision. Subtracting two absolute double-second stamps (~1.7e9) would
  // lose a few hundred ns to floating-point rounding.
  int64_t prev_nanoseconds = old_nanoseconds;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  for (auto twist_it = old_twist_it; twist_it != new_twist_it + 1; ++twist_it) {
    const int64_t current_nanoseconds = (twist_it != new_twist_it)
                                          ? utils::to_nanoseconds((*twist_it).header.stamp)
                                          : new_nanoseconds;
    const double dt = static_cast<double>(current_nanoseconds - prev_nanoseconds) * 1e-9;

    if (std::fabs(dt) > 0.1) {
      if (status != nullptr) status->twist_time_gap_too_large = true;
      break;
    }

    const double distance = (*twist_it).twist.linear.x * dt;
    yaw += (*twist_it).twist.angular.z * dt;
    x += distance * std::cos(yaw);
    y += distance * std::sin(yaw);
    prev_nanoseconds = utils::to_nanoseconds((*twist_it).header.stamp);
  }

  Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

  float cos_yaw = std::cos(yaw);
  float sin_yaw = std::sin(yaw);

  transformation_matrix(0, 3) = x;
  transformation_matrix(1, 3) = y;
  transformation_matrix(0, 0) = cos_yaw;
  transformation_matrix(0, 1) = -sin_yaw;
  transformation_matrix(1, 0) = sin_yaw;
  transformation_matrix(1, 1) = cos_yaw;

  return transformation_matrix;
}

}  // namespace autoware::pointcloud_preprocessor
