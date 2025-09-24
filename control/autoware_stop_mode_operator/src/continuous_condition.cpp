//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "continuous_condition.hpp"

namespace autoware::stop_mode_operator
{

void ContinuousCondition::update(const rclcpp::Time & stamp, bool condition)
{
  if (!condition) {
    start_stamp_ = std::nullopt;
  } else if (!start_stamp_) {
    start_stamp_ = stamp;
  }
}

void ContinuousCondition::update(const rclcpp::Time & stamp, double timeout)
{
  if (start_stamp_) {
    if (timeout <= (stamp - *start_stamp_).seconds()) {
      start_stamp_ = std::nullopt;
    }
  }
}

bool ContinuousCondition::check(const rclcpp::Time & stamp, double duration) const
{
  if (!start_stamp_) {
    return false;
  }
  return duration <= (stamp - *start_stamp_).seconds();
}

}  // namespace autoware::stop_mode_operator
