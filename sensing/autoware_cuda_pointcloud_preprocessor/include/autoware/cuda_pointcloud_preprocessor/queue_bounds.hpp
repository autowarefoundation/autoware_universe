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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__QUEUE_BOUNDS_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__QUEUE_BOUNDS_HPP_

#include <rclcpp/time.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace autoware::cuda_pointcloud_preprocessor::detail
{

template <typename MessageT>
std::uint64_t stamp_nsec(const MessageT & message)
{
  return static_cast<std::uint64_t>(rclcpp::Time(message.header.stamp).nanoseconds());
}

template <typename QueueT>
void prune_old_queue_entries(QueueT & queue, const std::uint64_t first_point_stamp)
{
  const auto queue_it = std::lower_bound(
    queue.begin(), queue.end(), first_point_stamp,
    [](const auto & message, const auto stamp) { return stamp_nsec(message) < stamp; });
  queue.erase(queue.begin(), queue_it);
}

template <typename MessagePtrsT>
void sort_and_prune_old_messages(MessagePtrsT & messages, const std::uint64_t first_point_stamp)
{
  const auto messages_it = std::remove_if(
    messages.begin(), messages.end(),
    [first_point_stamp](const auto & message) { return stamp_nsec(*message) < first_point_stamp; });
  messages.erase(messages_it, messages.end());

  std::stable_sort(messages.begin(), messages.end(), [](const auto & lhs, const auto & rhs) {
    return stamp_nsec(*lhs) < stamp_nsec(*rhs);
  });
}

template <typename QueueT, typename MessagePtrsT>
std::size_t prepare_queue_update(
  QueueT & queue, MessagePtrsT & messages, const std::size_t max_queue_size,
  const std::uint64_t first_point_stamp)
{
  if (queue.size() > max_queue_size) {
    throw std::runtime_error("Internal pointcloud preprocessor queue already exceeds capacity");
  }

  prune_old_queue_entries(queue, first_point_stamp);
  sort_and_prune_old_messages(messages, first_point_stamp);

  const auto free_capacity = max_queue_size - queue.size();
  if (messages.size() <= free_capacity) {
    return 0U;
  }

  const auto dropped_count = messages.size() - free_capacity;
  messages.erase(messages.begin(), messages.end() - free_capacity);
  return dropped_count;
}

constexpr std::uint64_t max_backward_time_jump_nsec = 1'000'000'000UL;

template <typename QueueT, typename StampT>
bool is_backward_time_jump(const QueueT & queue, const StampT & incoming_stamp)
{
  if (queue.empty()) {
    return false;
  }

  const auto queue_front_stamp = stamp_nsec(queue.front());
  const auto incoming_stamp_nsec =
    static_cast<std::uint64_t>(rclcpp::Time(incoming_stamp).nanoseconds());
  return queue_front_stamp > incoming_stamp_nsec + max_backward_time_jump_nsec;
}

template <typename QueueT, typename MessageT>
void insert_sorted(QueueT & queue, const MessageT & message)
{
  const auto it = std::lower_bound(
    queue.begin(), queue.end(), message.header.stamp,
    [](const auto & queued_message, const auto & stamp) {
      return rclcpp::Time(queued_message.header.stamp) < stamp;
    });
  queue.insert(it, message);
}

}  // namespace autoware::cuda_pointcloud_preprocessor::detail

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__QUEUE_BOUNDS_HPP_
