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

#include "status.hpp"

#include <vector>

namespace autoware::driving_mode_manager
{

bool TimeoutStatus::timeout() const
{
  return stamp_ == std::nullopt;
}

bool TimeoutStatus::status() const
{
  return stamp_ ? value_ : false;
}

void TimeoutStatus::update(const rclcpp::Time & now, bool status)
{
  stamp_ = now;
  value_ = status;
}

void TimeoutStatus::update(const rclcpp::Time & now, double timeout)
{
  if (stamp_ && timeout < (now - stamp_.value()).seconds()) {
    stamp_ = std::nullopt;
    value_ = false;
  }
}

DrivingModeStatus::DrivingModeStatus(const std::vector<AutowareMode> & modes)
{
  for (const auto & mode : modes) {
    modes_[mode.id] = DrivingModeStatusData{};
  }
}

void DrivingModeStatus::update(const rclcpp::Time & now, double timeout)
{
  for (auto & [id, status] : modes_) {
    status.available.update(now, timeout);
    status.stable.update(now, timeout);
    status.continuable.update(now, timeout);
  }
}

DrivingModeStatusData * DrivingModeStatus::data(const AutowareMode & mode)
{
  const auto iter = modes_.find(mode.id);
  return iter == modes_.end() ? nullptr : &iter->second;
}

bool DrivingModeStatus::is_ready() const
{
  for (const auto & [id, status] : modes_) {
    if (status.available.timeout()) return false;
    if (status.stable.timeout()) return false;
    if (status.continuable.timeout()) return false;
  }
  return true;
}

bool DrivingModeStatus::is_available(const AutowareMode & mode) const
{
  const auto iter = modes_.find(mode.id);
  return iter == modes_.end() ? false : iter->second.available.status();
}

bool DrivingModeStatus::is_stable(const AutowareMode & mode) const
{
  const auto iter = modes_.find(mode.id);
  return iter == modes_.end() ? false : iter->second.stable.status();
}

bool DrivingModeStatus::is_continuable(const AutowareMode & mode) const
{
  const auto iter = modes_.find(mode.id);
  return iter == modes_.end() ? false : iter->second.continuable.status();
}

}  // namespace autoware::driving_mode_manager
