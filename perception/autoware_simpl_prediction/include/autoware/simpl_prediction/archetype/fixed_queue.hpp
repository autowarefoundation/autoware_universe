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

#ifndef AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__FIXED_QUEUE_HPP_
#define AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__FIXED_QUEUE_HPP_

#include <deque>

namespace autoware::simpl_prediction::archetype
{
/**
 * @brief Fixed size of the queue data container.
 *
 * @tparam T Data type of elements in the queue.
 */
template <typename T>
class FixedQueue
{
public:
  using size_type = typename std::deque<T>::size_type;
  using reference = typename std::deque<T>::reference;
  using const_reference = typename std::deque<T>::const_reference;
  using iterator = typename std::deque<T>::iterator;
  using const_iterator = typename std::deque<T>::const_iterator;

  /**
   * @brief Construct a new FixedQueue object
   *
   * @param size Maximum number of elements can be contained in the queue.
   */
  explicit FixedQueue(size_type size) : queue_(size) {}

  /**
   * @brief Add data to the end of the queue removing the first element.
   *
   * @param t Data to be added.
   */
  void push_back(const T && t) noexcept
  {
    queue_.pop_front();
    queue_.push_back(t);
  }

  /**
   * @brief Add data to the end of the queue removing the first element.
   *
   * @param t Data to be added.
   */
  void push_back(const T & t) noexcept
  {
    queue_.pop_front();
    queue_.push_back(t);
  }

  /**
   * @brief Return a read/write reference to the data at the first element of the queue.
   */
  reference front() noexcept { return queue_.front(); }

  /**
   * @brief Return a read only reference to the data at the first element of the queue.
   */
  const_reference front() const noexcept { return queue_.front(); }

  /**
   * @brief Return a read/write reference to the data at the last element of the queue.
   */
  reference back() noexcept { return queue_.back(); }

  /**
   * @brief Return a read only reference to the data at the last element of the queue.
   */
  const_reference back() const noexcept { return queue_.back(); }

  /**
   * @brief Return the read/write iterator that points to the first element of the queue.
   */
  iterator begin() noexcept { return queue_.begin(); }

  /**
   * @brief Return the read only iterator that points to the first element of the queue.
   */
  const_iterator begin() const noexcept { return queue_.begin(); }

  /**
   * @brief Return the read/write iterator that points to the last element of the queue.
   */
  iterator end() noexcept { return queue_.end(); }

  /**
   * @brief Return the read only iterator that points to the first element of the queue.
   */
  const_iterator end() const noexcept { return queue_.end(); }

  /**
   * @brief Return the read/write reference to the data at the specified element of the queue.
   *
   * @param n Index of the element.
   */
  reference at(size_type n) { return queue_.at(n); }

  /**
   * @brief Return the read only reference to the data at the specified element of the queue.
   *
   * @param n Index of the element.
   */
  const_reference at(size_type n) const { return queue_.at(n); }

  /**
   * @brief Return the number of elements of the queue.
   */
  size_type size() const noexcept { return queue_.size(); }

private:
  std::deque<T> queue_;  //!< Queue data container.
};
}  // namespace autoware::simpl_prediction::archetype
#endif  // AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__FIXED_QUEUE_HPP_
