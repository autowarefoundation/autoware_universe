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

#ifndef TRAJECTORY_GATE_HPP_
#define TRAJECTORY_GATE_HPP_

#include "core/interface.hpp"
#include "core/selector.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/trajectory_source_status.hpp>
#include <tier4_system_msgs/srv/change_trajectory_source.hpp>

#include <memory>
#include <vector>

namespace autoware::trajectory_gate
{

class TrajectoryGate : public rclcpp::Node
{
public:
  explicit TrajectoryGate(const rclcpp::NodeOptions & options);

private:
  using TrajectorySourceStatus = tier4_system_msgs::msg::TrajectorySourceStatus;
  using ChangeTrajectorySource = tier4_system_msgs::srv::ChangeTrajectorySource;

  diagnostic_updater::Updater diag_;

  TrajectorySelector selector_;
  std::vector<std::unique_ptr<TrajectorySender>> subscriptions_;
  std::vector<std::unique_ptr<TrajectoryReceiver>> receivers_;

  rclcpp::Publisher<TrajectorySourceStatus>::SharedPtr pub_source_;
  rclcpp::Service<ChangeTrajectorySource>::SharedPtr srv_source_;

  void publish_source() const;
  void on_change_source(
    const ChangeTrajectorySource::Request::SharedPtr req,
    const ChangeTrajectorySource::Response::SharedPtr res);
};

}  // namespace autoware::trajectory_gate

#endif  // TRAJECTORY_GATE_HPP_
