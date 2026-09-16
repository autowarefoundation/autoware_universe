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

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace autoware::cuda_pointcloud_preprocessor::detail
{

/// \brief Nanoseconds since epoch of a builtin_interfaces/Time.
/// \throws std::runtime_error if `stamp.sec` is negative, which would otherwise wrap.
template <typename StampT>
std::uint64_t to_nsec(const StampT & stamp)
{
  if (stamp.sec < 0) {
    throw std::runtime_error("Message stamp precedes the epoch");
  }

  return static_cast<std::uint64_t>(stamp.sec) * 1'000'000'000ULL + stamp.nanosec;
}

template <typename MessageT>
std::uint64_t stamp_nsec(const MessageT & message)
{
  return to_nsec(message.header.stamp);
}

template <typename QueueT>
bool is_sorted_by_stamp(const QueueT & queue)
{
  return std::is_sorted(queue.begin(), queue.end(), [](const auto & lhs, const auto & rhs) {
    return stamp_nsec(lhs) < stamp_nsec(rhs);
  });
}

/// \brief Erases the entries older than `first_point_stamp`. `queue` must be sorted by stamp, and
/// dropping a prefix keeps it that way.
template <typename QueueT>
void prune_old_queue_entries(QueueT & queue, const std::uint64_t first_point_stamp)
{
  assert(is_sorted_by_stamp(queue));

  const auto queue_it = std::lower_bound(
    queue.begin(), queue.end(), first_point_stamp,
    [](const auto & message, const auto stamp) { return stamp_nsec(message) < stamp; });
  queue.erase(queue.begin(), queue_it);

  assert(is_sorted_by_stamp(queue));
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

/**
 * \brief Prunes the entries older than `first_point_stamp` from both containers, sorts `messages`
 * by stamp, then drops the oldest of them until they fit the capacity `queue` leaves free. Both are
 * left sorted by stamp, with `queue.size() + messages.size() <= max_queue_size`.
 * \return the number of messages dropped for capacity.
 * \throws std::runtime_error if `queue` already exceeds `max_queue_size`.
 */
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

  return stamp_nsec(queue.front()) > to_nsec(incoming_stamp) + max_backward_time_jump_nsec;
}

template <typename QueueT, typename MessageT>
void insert_sorted(QueueT & queue, const MessageT & message)
{
  const auto it = std::lower_bound(
    queue.begin(), queue.end(), stamp_nsec(message),
    [](const auto & queued_message, const auto stamp) {
      return stamp_nsec(queued_message) < stamp;
    });
  queue.insert(it, message);
}

}  // namespace autoware::cuda_pointcloud_preprocessor::detail

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__QUEUE_BOUNDS_HPP_
