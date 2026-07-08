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

#include "autoware/diffusion_planner/preprocessing/ego_history.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{
namespace
{
using nav_msgs::msg::Odometry;

// Linearly interpolate pose and twist between two samples; ratio 0 returns `earlier`, 1 `later`.
Odometry interpolate_ego_state(const Odometry & earlier, const Odometry & later, const double ratio)
{
  Odometry interpolated = earlier;
  interpolated.pose.pose = autoware_utils_geometry::calc_interpolated_pose(
    earlier.pose.pose, later.pose.pose, ratio, false);
  const auto lerp = [ratio](const double a, const double b) { return a + (b - a) * ratio; };
  auto & lin = interpolated.twist.twist.linear;
  auto & ang = interpolated.twist.twist.angular;
  lin.x = lerp(earlier.twist.twist.linear.x, later.twist.twist.linear.x);
  lin.y = lerp(earlier.twist.twist.linear.y, later.twist.twist.linear.y);
  lin.z = lerp(earlier.twist.twist.linear.z, later.twist.twist.linear.z);
  ang.x = lerp(earlier.twist.twist.angular.x, later.twist.twist.angular.x);
  ang.y = lerp(earlier.twist.twist.angular.y, later.twist.twist.angular.y);
  ang.z = lerp(earlier.twist.twist.angular.z, later.twist.twist.angular.z);
  return interpolated;
}
}  // namespace

EgoHistory::EgoHistory(const double buffer_window_s) : buffer_window_s_(buffer_window_s)
{
}

void EgoHistory::update(
  const std::vector<std::shared_ptr<const nav_msgs::msg::Odometry>> & ego_states)
{
  // Append raw odometry, keeping the buffer monotonic oldest-to-newest. Dropping buffered samples
  // not older than an incoming stamp keeps back() the true newest even across a bag loop, sim
  // reset, or out-of-order stamp, so the window prune below and the interpolation in
  // select_state()/to_agent_past() stay valid.
  for (const auto & ego_state : ego_states) {
    if (!ego_state) {
      continue;
    }
    const rclcpp::Time incoming_stamp(ego_state->header.stamp);
    while (!buffer_.empty() && rclcpp::Time(buffer_.back().header.stamp) >= incoming_stamp) {
      buffer_.pop_back();
    }
    buffer_.push_back(*ego_state);
  }

  // Prune to the time window (the buffer is now monotonic, so back() is the newest sample).
  if (!buffer_.empty()) {
    const rclcpp::Time newest_stamp(buffer_.back().header.stamp);
    while (buffer_.size() > 1 &&
           (newest_stamp - rclcpp::Time(buffer_.front().header.stamp)).seconds() >
             buffer_window_s_) {
      buffer_.pop_front();
    }
  }
}

rclcpp::Time EgoHistory::newest_stamp() const
{
  return rclcpp::Time(buffer_.back().header.stamp);
}

std::pair<nav_msgs::msg::Odometry, double> EgoHistory::select_state(
  const rclcpp::Time & frame_time, const bool use_time_interpolation) const
{
  return select_ego_state(buffer_, frame_time, use_time_interpolation);
}

std::vector<float> EgoHistory::to_agent_past(
  const Eigen::Matrix4d & map_to_ego_transform, const rclcpp::Time & reference_time,
  const bool use_time_interpolation, const bool shift_x, const double base_link_to_center) const
{
  // The buffer holds raw odometry; apply the same vehicle-center shift as the reference transform
  // so the past trajectory is expressed in the current ego frame.
  if (!shift_x) {
    return create_ego_agent_past(
      buffer_, EGO_HISTORY_SHAPE[1], map_to_ego_transform, reference_time, use_time_interpolation);
  }
  std::deque<nav_msgs::msg::Odometry> shifted = buffer_;
  for (auto & odom : shifted) {
    odom.pose.pose = utils::shift_x(odom.pose.pose, base_link_to_center);
  }
  return create_ego_agent_past(
    shifted, EGO_HISTORY_SHAPE[1], map_to_ego_transform, reference_time, use_time_interpolation);
}

std::pair<nav_msgs::msg::Odometry, double> EgoHistory::select_ego_state(
  const std::deque<Odometry> & buffer, const rclcpp::Time & frame_time,
  const bool use_time_interpolation)
{
  // Nearest sample: used when interpolation is off, and as the fallback outside the buffer range.
  const Odometry * nearest = &buffer.front();
  double min_time_diff_s = std::numeric_limits<double>::max();
  for (const auto & candidate : buffer) {
    const double time_diff_s =
      std::abs((rclcpp::Time(candidate.header.stamp) - frame_time).seconds());
    if (time_diff_s < min_time_diff_s) {
      min_time_diff_s = time_diff_s;
      nearest = &candidate;
    }
  }

  if (!use_time_interpolation) {
    return {*nearest, min_time_diff_s};
  }

  const double frame_sec = frame_time.seconds();
  for (size_t i = 0; i + 1 < buffer.size(); ++i) {
    const double t0 = rclcpp::Time(buffer[i].header.stamp).seconds();
    const double t1 = rclcpp::Time(buffer[i + 1].header.stamp).seconds();
    if (frame_sec >= t0 && frame_sec <= t1) {
      const double ratio = (t1 > t0) ? (frame_sec - t0) / (t1 - t0) : 0.0;
      return {interpolate_ego_state(buffer[i], buffer[i + 1], ratio), 0.0};
    }
  }
  // frame_time is outside the buffered range: fall back to the nearest sample.
  return {*nearest, min_time_diff_s};
}

std::vector<float> EgoHistory::create_ego_agent_past(
  const std::deque<nav_msgs::msg::Odometry> & buffer, size_t num_timesteps,
  const Eigen::Matrix4d & map_to_ego_transform, const std::optional<rclcpp::Time> & reference_time,
  bool use_time_interpolation)
{
  const size_t features_per_timestep = 4;  // x, y, cos, sin
  const size_t total_size = num_timesteps * features_per_timestep;

  std::vector<float> ego_agent_past(total_size, 0.0f);

  // Initialize cos values to 1.0 (identity heading)
  for (size_t t = 0; t < num_timesteps; ++t) {
    ego_agent_past[t * features_per_timestep + EGO_AGENT_PAST_IDX_COS] = 1.0f;
  }

  // If no odometry messages are available, return the default-initialized ego_agent_past
  if (buffer.empty()) {
    return ego_agent_past;
  }

  // Lambda to transform a pose to ego frame and store into the flat array
  auto store_pose = [&](size_t timestep_idx, const geometry_msgs::msg::Pose & pose) {
    const Eigen::Matrix4d pose_map_4x4 = utils::pose_to_matrix4d(pose);
    const Eigen::Matrix4d pose_ego_4x4 = map_to_ego_transform * pose_map_4x4;
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(pose_ego_4x4.block<3, 3>(0, 0));
    const size_t base_idx = timestep_idx * features_per_timestep;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_X] = pose_ego_4x4(0, 3);
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_Y] = pose_ego_4x4(1, 3);
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_COS] = cos_yaw;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_SIN] = sin_yaw;
  };

  if (!reference_time.has_value()) {
    // Legacy behavior: use the last num_timesteps odom messages directly
    const size_t start_idx = (buffer.size() >= num_timesteps) ? buffer.size() - num_timesteps : 0;
    for (size_t i = start_idx; i < buffer.size(); ++i) {
      store_pose(i - start_idx, buffer[i].pose.pose);
    }
    return ego_agent_past;
  }

  // Time-based interpolation behavior
  auto stamp_to_sec = [](const builtin_interfaces::msg::Time & stamp) -> double {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  };

  const double ref_sec = reference_time->seconds();
  constexpr double dt = constants::PREDICTION_TIME_STEP_S;  // 0.1s

  const double first_sec = stamp_to_sec(buffer.front().header.stamp);
  const double last_sec = stamp_to_sec(buffer.back().header.stamp);

  // Both target_sec and odom timestamps are monotonically increasing,
  // so we can carry the search index forward across iterations.
  size_t search_start = 0;

  for (size_t t = 0; t < num_timesteps; ++t) {
    // t=0 is the oldest, t=num_timesteps-1 is the reference time
    const double target_sec = ref_sec - static_cast<double>(num_timesteps - 1 - t) * dt;

    geometry_msgs::msg::Pose interpolated_pose;
    if (target_sec <= first_sec) {
      interpolated_pose = buffer.front().pose.pose;
    } else if (target_sec >= last_sec) {
      interpolated_pose = buffer.back().pose.pose;
    } else {
      // Find the two bracketing odom messages, continuing from previous position
      for (; search_start + 1 < buffer.size(); ++search_start) {
        const double t_next = stamp_to_sec(buffer[search_start + 1].header.stamp);
        if (target_sec <= t_next) {
          break;
        }
      }

      const double t0 = stamp_to_sec(buffer[search_start].header.stamp);
      const double t1 = stamp_to_sec(buffer[search_start + 1].header.stamp);
      const double ratio = (t1 > t0) ? (target_sec - t0) / (t1 - t0) : 0.0;

      if (use_time_interpolation) {
        interpolated_pose = autoware_utils_geometry::calc_interpolated_pose(
          buffer[search_start].pose.pose, buffer[search_start + 1].pose.pose, ratio, false);
      } else {
        // Match closest: take whichever bracketing message is nearer to the target time.
        interpolated_pose =
          (ratio < 0.5) ? buffer[search_start].pose.pose : buffer[search_start + 1].pose.pose;
      }
    }

    store_pose(t, interpolated_pose);
  }

  return ego_agent_past;
}

}  // namespace autoware::diffusion_planner::preprocess
