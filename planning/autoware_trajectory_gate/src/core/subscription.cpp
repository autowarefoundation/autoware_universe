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

#include "subscription.hpp"

#include <string>

namespace autoware::trajectory_gate
{

TrajectorySubscription::TrajectorySubscription(const std::string & name, rclcpp::Node & node)
{
  using std::placeholders::_1;

  sub_trajectory_ = node.create_subscription<Trajectory>(
    "~/inputs/" + name + "/trajectory", rclcpp::QoS(1),
    std::bind(&TrajectorySubscription::on_msg, this, _1));
}

void TrajectorySubscription::on_msg(const Trajectory & msg)
{
  TrajectorySender::send(msg);
}

}  // namespace autoware::trajectory_gate
