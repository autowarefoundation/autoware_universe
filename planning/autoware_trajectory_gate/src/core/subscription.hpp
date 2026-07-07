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

#ifndef CORE__SUBSCRIPTION_HPP_
#define CORE__SUBSCRIPTION_HPP_

#include "interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::trajectory_gate
{

class TrajectorySubscription : public TrajectorySender
{
public:
  TrajectorySubscription(const std::string & name, rclcpp::Node & node);

private:
  void on_msg(const Trajectory & msg);
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
};

}  // namespace autoware::trajectory_gate

#endif  // CORE__SUBSCRIPTION_HPP_
