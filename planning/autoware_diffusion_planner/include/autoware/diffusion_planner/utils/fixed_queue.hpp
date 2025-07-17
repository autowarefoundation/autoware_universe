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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__FIXED_QUEUE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__FIXED_QUEUE_HPP_

#include <cstddef>
#include <deque>
#include <iostream>
#include <iterator>

namespace autoware::diffusion_planner
{

/**
 * @brief A fixed-size queue that discards the oldest elements when full.
 *
 * This queue behaves like a standard deque, but when the maximum size is reached,
 * pushing a new element will remove the oldest element from the opposite end.
 *
 * @tparam T The type of elements stored in the queue.
 */
template <typename T>
class FixedQueue
{
public:
  using size_type = typename std::deque<T>::size_type;
  using reference = typename std::deque<T>::reference;
  using const_reference = typename std::deque<T>::const_reference;
  using iterator = typename std::deque<T>::iterator;
  using reverse_iterator = typename std::reverse_iterator<iterator>;
  using const_iterator = typename std::deque<T>::const_iterator;
  using const_reverse_iterator = typename std::reverse_iterator<const_iterator>;

  /**
   * @brief Construct a FixedQueue with a maximum size.
   * @param size The maximum number of elements the queue can hold.
   */
  explicit FixedQueue(size_type size) : max_size_(size) {}

  /**
   * @brief Push an rvalue element to the back. Removes front if full.
   */
  void push_back(const T && t) noexcept
  {
    if (queue_.size() >= max_size_) {
      queue_.pop_front();
    }
    queue_.push_back(t);
  }

  /**
   * @brief Push an lvalue element to the back. Removes front if full.
   */
  void push_back(const T & t) noexcept
  {
    if (queue_.size() >= max_size_) {
      queue_.pop_front();
    }
    queue_.push_back(t);
  }

  /**
   * @brief Push an rvalue element to the front. Removes back if full.
   */
  void push_front(const T && t) noexcept
  {
    if (queue_.size() >= max_size_) {
      queue_.pop_back();
    }
    queue_.push_front(t);
  }

  /**
   * @brief Push an lvalue element to the front. Removes back if full.
   */
  void push_front(const T & t) noexcept
  {
    if (queue_.size() >= max_size_) {
      queue_.pop_back();
    }
    queue_.push_front(t);
  }

  /**
   * @brief Access the first element.
   */
  reference front() noexcept { return queue_.front(); }
  const_reference front() const noexcept { return queue_.front(); }

  /**
   * @brief Access the last element.
   */
  reference back() noexcept { return queue_.back(); }
  const_reference back() const noexcept { return queue_.back(); }

  /**
   * @brief Iterator to the beginning.
   */
  iterator begin() noexcept { return queue_.begin(); }
  const_iterator begin() const noexcept { return queue_.begin(); }

  /**
   * @brief Iterator to the end.
   */
  iterator end() noexcept { return queue_.end(); }
  const_iterator end() const noexcept { return queue_.end(); }

  /**
   * @brief Reverse iterator to the beginning.
   */
  reverse_iterator rbegin() noexcept { return queue_.rbegin(); }
  const_reverse_iterator rbegin() const noexcept { return queue_.rbegin(); }

  /**
   * @brief Reverse iterator to the end.
   */
  reverse_iterator rend() noexcept { return queue_.rend(); }
  const_reverse_iterator rend() const noexcept { return queue_.rend(); }

  /**
   * @brief Returns the number of elements in the queue.
   */
  size_type size() const noexcept { return queue_.size(); }

private:
  std::deque<T> queue_;  ///< Underlying container.
  size_t max_size_{0};   ///< Maximum allowed size.
};
}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__FIXED_QUEUE_HPP_
