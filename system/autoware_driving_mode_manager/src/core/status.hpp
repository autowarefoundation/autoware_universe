// Copyright 2026 The Autoware Contributors
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

#ifndef CORE__STATUS_HPP_
#define CORE__STATUS_HPP_

#include <autoware_driving_mode_manager/types.hpp>
#include <rclcpp/time.hpp>

#include <optional>
#include <unordered_map>
#include <vector>

namespace autoware::driving_mode_manager
{

class TimeoutStatus
{
public:
  bool timeout() const;
  bool status() const;
  void update(const rclcpp::Time & now, bool status);
  void update(const rclcpp::Time & now, double timeout);

private:
  std::optional<rclcpp::Time> stamp_ = std::nullopt;
  bool value_ = false;
};

struct DrivingModeStatusData
{
  TimeoutStatus available;
  TimeoutStatus active;
  TimeoutStatus stable;
  TimeoutStatus continuable;
};

class DrivingModeStatus
{
public:
  explicit DrivingModeStatus(const std::vector<AutowareMode> & modes);
  void update(const rclcpp::Time & now, double timeout);
  DrivingModeStatusData * data(const AutowareMode & mode);

  bool is_ready() const;
  bool is_available(const AutowareMode & mode) const;
  bool is_active(const AutowareMode & mode) const;
  bool is_stable(const AutowareMode & mode) const;
  bool is_continuable(const AutowareMode & mode) const;

private:
  std::unordered_map<uint32_t, DrivingModeStatusData> modes_;
};

}  // namespace autoware::driving_mode_manager

#endif  // CORE__STATUS_HPP_
