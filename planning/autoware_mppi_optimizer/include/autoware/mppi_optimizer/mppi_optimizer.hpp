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

#ifndef AUTOWARE__MPPI_OPTIMIZER__MPPI_OPTIMIZER_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__MPPI_OPTIMIZER_HPP_

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

namespace autoware::mppi_optimizer
{

class FirstOrderDubinsMppiInterface;

using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

class MppiOptimizer : public rclcpp::Node
{
public:
  explicit MppiOptimizer(const rclcpp::NodeOptions & options);

private:
  void on_trajectory(const Trajectory::ConstSharedPtr msg);

  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;

  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_odometry_{this, "~/input/odometry"};

  std::unique_ptr<FirstOrderDubinsMppiInterface> mppi_interface_;
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__MPPI_OPTIMIZER_HPP_
