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
 * @brief Enum representing left or right side.
 */
enum class SideKey { LEFT, RIGHT };

/**
 * @brief Convert SideKey to string.
 * @param[in] key The side key.
 * @return "left", "right", or "unknown".
 */
inline std::string to_string(const SideKey key)
{
  if (key == SideKey::LEFT) return "left";
  if (key == SideKey::RIGHT) return "right";
  return "unknown";
}

/**
 * @brief Type trait to check if a type has an empty() method.
 * @tparam T The type to check.
 */
template <typename T, typename = void>
struct HasEmpty : std::false_type
{
};

/**
 * @brief Specialization for types having empty() method.
 */
template <typename T>
struct HasEmpty<T, std::void_t<decltype(std::declval<T>().empty())>> : std::true_type
{
};

/**
 * @brief Template structure to hold data for both left and right sides.
 * @tparam T The type of data to hold for each side.
 */
template <typename T>
struct Side
{
  /**
   * @brief Access data for a specific side.
   * @param[in] key The side key.
   * @return Reference to the data on that side.
   * @throws std::invalid_argument If key is invalid.
   */
  T & operator[](const SideKey key)
  {
    if (key == SideKey::LEFT) return left;
    if (key == SideKey::RIGHT) return right;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }

  /**
   * @brief Access data for a specific side (const).
   * @param[in] key The side key.
   * @return Const reference to the data on that side.
   * @throws std::invalid_argument If key is invalid.
   */
  const T & operator[](const SideKey key) const
  {
    if (key == SideKey::LEFT) return left;
    if (key == SideKey::RIGHT) return right;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }

  /**
   * @brief Execute a function for each side, passing side key as integral constant.
   * @tparam Func Function type.
   * @param[in] fn Function to execute.
   */
  template <typename Func>
  void for_each(Func && fn)
  {
    fn(std::integral_constant<SideKey, SideKey::LEFT>{}, left);
    fn(std::integral_constant<SideKey, SideKey::RIGHT>{}, right);
  }

  /**
   * @brief Execute a function for each side, passing side key as integral constant (const).
   * @tparam Func Function type.
   * @param[in] fn Function to execute.
   */
  template <typename Func>
  void for_each(Func && fn) const
  {
    fn(std::integral_constant<SideKey, SideKey::LEFT>{}, left);
    fn(std::integral_constant<SideKey, SideKey::RIGHT>{}, right);
  }

  /**
   * @brief Execute a function for each side's data.
   * @tparam Func Function type.
   * @param[in] fn Function to execute.
   */
  template <typename Func>
  void for_each_side(Func && fn)
  {
    fn(left);
    fn(right);
  }

  /**
   * @brief Execute a function for each side's data (const).
   * @tparam Func Function type.
   * @param[in] fn Function to execute.
   */
  template <typename Func>
  void for_each_side(Func && fn) const
  {
    fn(left);
    fn(right);
  }

  /**
   * @brief Transform data on each side using a function and return a new Side object.
   * @tparam Func Function type.
   * @param[in] fn Transformation function.
   * @return A Side object containing the transformed data.
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
   * @brief Check if data on both sides is empty.
   * @tparam U Helper type for enable_if.
   * @return True if both sides are empty.
   */
  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  [[nodiscard]] bool all_empty() const
  {
    return left.empty() && right.empty();
  }

  /**
   * @brief Reserve capacity for both sides.
   * @tparam U Helper type for enable_if.
   * @param[in] size Capacity to reserve.
   */
  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  void reserve_all(const size_t size)
  {
    left.reserve(size);
    right.reserve(size);
  }

  /**
   * @brief Check if both sides have the same number of elements.
   * @tparam U Helper type for enable_if.
   * @return True if sizes are equal.
   */
  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  [[nodiscard]] bool equal_size() const
  {
    return left.size() == right.size();
  }

  /**
   * @brief Get the maximum size among both sides.
   * @tparam U Helper type for enable_if.
   * @return Maximum size.
   */
  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  [[nodiscard]] size_t max_size() const
  {
    return std::max(left.size(), right.size());
  }

  /**
   * @brief Check if any side satisfies a predicate.
   * @tparam Func Predicate type.
   * @param[in] fn Predicate function.
   * @return True if either side satisfies the predicate.
   */
  template <typename Func>
  [[nodiscard]] bool any_of_side(Func && fn) const
  {
    return fn(left) || fn(right);
  }

  /**
   * @brief Check if all sides satisfy a predicate.
   * @tparam Func Predicate type.
   * @param[in] fn Predicate function.
   * @return True if both sides satisfy the predicate.
   */
  template <typename Func>
  [[nodiscard]] bool all_of_side(Func && fn) const
  {
    return fn(left) && fn(right);
  }

  // Member Variables
  /**
   * @brief Data for the right side.
   */
  T right;

  /**
   * @brief Data for the left side.
   */
  T left;
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__SIDE_STRUCT_HPP_
