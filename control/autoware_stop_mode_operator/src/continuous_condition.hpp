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

#ifndef CONTINUOUS_CONDITION_HPP_
#define CONTINUOUS_CONDITION_HPP_

#include <rclcpp/time.hpp>

#include <optional>

namespace autoware::stop_mode_operator
{

class ContinuousCondition
{
public:
  void update(const rclcpp::Time & stamp, bool condition);
  void update(const rclcpp::Time & stamp, double timeout);
  bool check(const rclcpp::Time & stamp, double duration) const;

private:
  std::optional<rclcpp::Time> start_stamp_;
};

}  // namespace autoware::stop_mode_operator

#endif  // CONTINUOUS_CONDITION_HPP_
