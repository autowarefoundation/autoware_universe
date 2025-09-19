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

#ifndef AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTER_CONTEXT_HPP_
#define AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTER_CONTEXT_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <any>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace autoware::trajectory_safety_filter
{

// Base context that all filters can access
struct FilterContext
{
  // Common data that most filters need
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_twist;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry;

  // Plugin-specific data storage
  std::unordered_map<std::string, std::any> custom_data;

  // Helper methods for type-safe access to custom data
  template <typename T>
  void set_custom_data(const std::string & key, const T & value)
  {
    custom_data[key] = value;
  }

  template <typename T>
  std::optional<T> get_custom_data(const std::string & key) const
  {
    auto it = custom_data.find(key);
    if (it != custom_data.end()) {
      try {
        return std::any_cast<T>(it->second);
      } catch (const std::bad_any_cast &) {
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  bool has_custom_data(const std::string & key) const
  {
    return custom_data.find(key) != custom_data.end();
  }
};

// Removed FilterParameters class - no longer needed with simplified design

}  // namespace autoware::trajectory_safety_filter

#endif  // AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTER_CONTEXT_HPP_
