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

#ifndef PATH_LENGTH_BUFFER_HPP_
#define PATH_LENGTH_BUFFER_HPP_

#include "types.hpp"

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner
{

struct BufferedStopDistanceItem
{
  double stop_distance;
  geometry_msgs::msg::Point stop_point;
  rclcpp::Time start_time;
  bool is_active;
  StopObstacle determined_stop_obstacle;
  double determined_desired_stop_margin;

  BufferedStopDistanceItem(
    double stop_distance, const geometry_msgs::msg::Point & point, const rclcpp::Time & time,
    bool active, const StopObstacle & determined_stop_obstacle,
    double determined_desired_stop_margin)
  : stop_distance(stop_distance),
    stop_point(point),
    start_time(time),
    is_active(active),
    determined_stop_obstacle(determined_stop_obstacle),
    determined_desired_stop_margin(determined_desired_stop_margin)
  {
  }

  bool should_activate(const rclcpp::Time & current_time, double min_on_duration) const
  {
    return !is_active && (current_time - start_time).seconds() > min_on_duration;
  }
};

class PathLengthBuffer
{
public:
  PathLengthBuffer() = default;
  PathLengthBuffer(
    double update_distance_threshold, double min_off_duration, double min_on_duration)
  : min_off_duration_(min_off_duration),
    min_on_duration_(min_on_duration),
    update_distance_th_(update_distance_threshold)
  {
  }

  std::optional<BufferedStopDistanceItem> get_nearest_active_item() const
  {
    if (buffer_.empty()) {
      return {};
    }

    auto nearest_item = std::min_element(
      buffer_.begin(), buffer_.end(),
      [](const BufferedStopDistanceItem & a, const BufferedStopDistanceItem & b) {
        if (!a.is_active) return false;
        if (!b.is_active) return true;
        return a.stop_distance < b.stop_distance;
      });

    if (!nearest_item->is_active) {
      return std::nullopt;
    }

    return *nearest_item;
  }

  void update_buffer(
    const std::optional<geometry_msgs::msg::Point> & stop_point,
    std::function<double(const geometry_msgs::msg::Point &)> calc_stop_distance,
    const rclcpp::Time & current_time, const StopObstacle & determined_stop_obstacle,
    double determined_desired_stop_margin)
  {
    if (!stop_point) {
      return;
    }

    const double stop_distance = calc_stop_distance(*stop_point);

    // Update the state of buffered-items
    for (auto & buffered_item : buffer_) {
      // Update the stop distance
      buffered_item.stop_distance = calc_stop_distance(buffered_item.stop_point);

      // Handle timing-based activation
      const bool should_activate = buffered_item.should_activate(current_time, min_on_duration_);
      if (!should_activate) {
        continue;
      }

      buffered_item.is_active = true;
      buffered_item.start_time = current_time;
    }

    // Remove items that are invalid
    buffer_.erase(
      std::remove_if(
        buffer_.begin(), buffer_.end(),
        [&](const BufferedStopDistanceItem & buffered_item) {
          const auto duration = (current_time - buffered_item.start_time).seconds();

          const double rel_dist = std::abs(buffered_item.stop_distance - stop_distance);

          return (buffered_item.is_active && (duration > min_off_duration_)) ||
                 (!buffered_item.is_active && rel_dist > update_distance_th_);
        }),
      buffer_.end());

    if (buffer_.empty()) {
      buffer_.emplace_back(
        stop_distance, *stop_point, current_time, false, determined_stop_obstacle,
        determined_desired_stop_margin);
      return;
    }

    auto nearest_prev_pose_it = buffer_.end();
    auto min_relative_dist = std::numeric_limits<double>::max();
    for (auto it = buffer_.begin(); it < buffer_.end(); ++it) {
      const double rel_dist = it->stop_distance - stop_distance;

      if (std::abs(rel_dist) < update_distance_th_ && rel_dist < min_relative_dist) {
        nearest_prev_pose_it = it;
        min_relative_dist = rel_dist;
      }
    }

    if (nearest_prev_pose_it == buffer_.end()) {
      buffer_.emplace_back(
        stop_distance, *stop_point, current_time, false, determined_stop_obstacle,
        determined_desired_stop_margin);
      return;
    }

    // Update the nearest item
    if (min_relative_dist > 0) {
      nearest_prev_pose_it->stop_distance = stop_distance;
      nearest_prev_pose_it->stop_point = *stop_point;
      nearest_prev_pose_it->determined_stop_obstacle = determined_stop_obstacle;
      nearest_prev_pose_it->determined_desired_stop_margin = determined_desired_stop_margin;
    }

    if (nearest_prev_pose_it->is_active) {
      nearest_prev_pose_it->start_time = current_time;
    }
  }

private:
  std::vector<BufferedStopDistanceItem> buffer_;
  double min_off_duration_{0.0};
  double min_on_duration_{0.0};
  double update_distance_th_{0.0};
};

}  // namespace autoware::motion_velocity_planner

#endif  // PATH_LENGTH_BUFFER_HPP_
