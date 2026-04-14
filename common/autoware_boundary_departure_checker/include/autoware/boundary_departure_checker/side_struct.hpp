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
enum class SideKey { LEFT, RIGHT };

inline std::string to_string(const SideKey key)
{
  if (key == SideKey::LEFT) return "left";
  if (key == SideKey::RIGHT) return "right";
  return "unknown";
}

template <typename T, typename = void>
struct HasEmpty : std::false_type
{
};

template <typename T>
struct HasEmpty<T, std::void_t<decltype(std::declval<T>().empty())>> : std::true_type
{
};

template <typename T>
struct Side
{
  T right;
  T left;

  T & operator[](const SideKey key)
  {
    if (key == SideKey::LEFT) return left;
    if (key == SideKey::RIGHT) return right;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }

  const T & operator[](const SideKey key) const
  {
    if (key == SideKey::LEFT) return left;
    if (key == SideKey::RIGHT) return right;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }

  template <typename Func>
  void for_each(Func && fn)
  {
    fn(std::integral_constant<SideKey, SideKey::LEFT>{}, left);
    fn(std::integral_constant<SideKey, SideKey::RIGHT>{}, right);
  }

  template <typename Func>
  void for_each(Func && fn) const
  {
    fn(std::integral_constant<SideKey, SideKey::LEFT>{}, left);
    fn(std::integral_constant<SideKey, SideKey::RIGHT>{}, right);
  }

  template <typename Func>
  void for_each_side(Func && fn)
  {
    fn(left);
    fn(right);
  }

  template <typename Func>
  void for_each_side(Func && fn) const
  {
    fn(left);
    fn(right);
  }

  template <typename Func>
  auto transform_each_side(Func && fn) const
  {
    using ResultType = decltype(fn(left));
    Side<ResultType> result;
    result.left = fn(left);
    result.right = fn(right);
    return result;
  }

  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  [[nodiscard]] bool all_empty() const
  {
    return left.empty() && right.empty();
  }

  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  void reserve_all(const size_t size)
  {
    left.reserve(size);
    right.reserve(size);
  }

  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  [[nodiscard]] bool equal_size() const
  {
    return left.size() == right.size();
  }

  template <typename U = T, std::enable_if_t<HasEmpty<U>::value, int> = 0>
  [[nodiscard]] size_t max_size() const
  {
    return std::max(left.size(), right.size());
  }

  template <typename Func>
  [[nodiscard]] bool any_of_side(Func && fn) const
  {
    return fn(left) || fn(right);
  }

  template <typename Func>
  [[nodiscard]] bool all_of_side(Func && fn) const
  {
    return fn(left) && fn(right);
  }
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__SIDE_STRUCT_HPP_
