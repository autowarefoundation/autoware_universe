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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__SIDE_STRUCT_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__SIDE_STRUCT_HPP_

#include <magic_enum.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace autoware::boundary_departure_checker
{
/**
 * @brief Key for side (left or right).
 */
enum class SideKey { LEFT, RIGHT };

/**
 * @brief Convert SideKey to string.
 * @param[in] key side key
 * @return string representation ("left", "right", or "unknown")
 */
inline std::string to_string(const SideKey key)
{
  if (key == SideKey::LEFT) return "left";
  if (key == SideKey::RIGHT) return "right";
  return "unknown";
}

/**
 * @brief Helper to check if a type has an empty() method.
 */
template <typename T, typename = void>
struct HasEmpty : std::false_type
{
};

/**
 * @brief Specialization of HasEmpty for types with empty().
 */
template <typename T>
struct HasEmpty<T, std::void_t<decltype(std::declval<T>().empty())>> : std::true_type
{
};

/**
 * @brief Template struct to hold data for both sides.
 * @tparam T type of data to hold
 */
template <typename T>
struct Side
{
  /**
   * @brief Access element by SideKey.
   * @param[in] key side key
   * @return reference to the element
   */
  T & operator[](const SideKey key)
  {
    if (key == SideKey::LEFT) return left;
    if (key == SideKey::RIGHT) return right;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }

  /**
   * @brief Access element by SideKey (const version).
   * @param[in] key side key
   * @return const reference to the element
   */
  const T & operator[](const SideKey key) const
  {
    if (key == SideKey::LEFT) return left;
    if (key == SideKey::RIGHT) return right;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }

  /**
   * @brief Apply a function to each side with side key information.
   * @tparam Func function type
   * @param[in] fn function to apply
   */
  template <typename Func>
  void for_each(Func && fn)
  {
    fn(std::integral_constant<SideKey, SideKey::LEFT>{}, left);
    fn(std::integral_constant<SideKey, SideKey::RIGHT>{}, right);
  }

  /**
   * @brief Apply a function to each side with side key information (const version).
   * @tparam Func function type
   * @param[in] fn function to apply
   */
  template <typename Func>
  void for_each(Func && fn) const
  {
    fn(std::integral_constant<SideKey, SideKey::LEFT>{}, left);
    fn(std::integral_constant<SideKey, SideKey::RIGHT>{}, right);
  }

  /**
   * @brief Apply a function to each side.
   * @tparam Func function type
   * @param[in] fn function to apply
   */
  template <typename Func>
  void for_each_side(Func && fn)
  {
    fn(left);
    fn(right);
  }

  /**
   * @brief Apply a function to each side (const version).
   * @tparam Func function type
   * @param[in] fn function to apply
   */
  template <typename Func>
  void for_each_side(Func && fn) const
  {
    fn(left);
    fn(right);
  }

  /**
   * @brief Transform each side using a function and return a new Side object.
   * @tparam Func function type
   * @param[in] fn transformation function
   * @return new Side object with transformed elements
   */
  template <typename Func>
  auto transform_each_side(Func && fn) const
  {
    using ResultType = decltype(fn(left));
    Side<ResultType> result;
    result.left = fn(left);
    result.right = fn(right);
    return result;
  }

  /**
   * @brief Check if both sides are empty.
   * @return true if both sides are empty
   */
  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  [[nodiscard]] bool all_empty() const
  {
    return left.empty() && right.empty();
  }

  /**
   * @brief Reserve space on both sides.
   * @param[in] size size to reserve
   */
  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  void reserve_all(const size_t size)
  {
    left.reserve(size);
    right.reserve(size);
  }

  /**
   * @brief Check if both sides have the same size.
   * @return true if sizes are equal
   */
  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  [[nodiscard]] bool equal_size() const
  {
    return left.size() == right.size();
  }

  /**
   * @brief Get the maximum size among both sides.
   * @return maximum size
   */
  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  [[nodiscard]] size_t max_size() const
  {
    return std::max(left.size(), right.size());
  }

  /**
   * @brief Check if any side satisfies a condition.
   * @tparam Func condition function type
   * @param[in] fn condition function
   * @return true if any side satisfies the condition
   */
  template <typename Func>
  [[nodiscard]] bool any_of_side(Func && fn) const
  {
    return fn(left) || fn(right);
  }

  /**
   * @brief Check if all sides satisfy a condition.
   * @tparam Func condition function type
   * @param[in] fn condition function
   * @return true if all sides satisfy the condition
   */
  template <typename Func>
  [[nodiscard]] bool all_of_side(Func && fn) const
  {
    return fn(left) && fn(right);
  }

  T right;  ///< data for the right side
  T left;   ///< data for the left side
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__SIDE_STRUCT_HPP_
