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

#ifndef TRAJECTORY_DUMMY_GATE_NODE_HPP_
#define TRAJECTORY_DUMMY_GATE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <std_msgs/msg/bool.hpp>

#include <mutex>

namespace autoware::trajectory_dummy_gate
{

using autoware_internal_planning_msgs::msg::CandidateTrajectories;

// Dummy gate for e2e verification: passes the DiffusionPlanner candidate trajectories through
// untouched, or — while the external override is ON — forwards the generator_info with an empty
// trajectory set so the downstream concatenator drops the diffusion-planner trajectory and only
// the backup planner reaches the selector. Not used in normal operation.
class TrajectoryDummyGateNode : public rclcpp::Node
{
public:
  explicit TrajectoryDummyGateNode(const rclcpp::NodeOptions & node_options);

private:
  void on_trajectories(const CandidateTrajectories::ConstSharedPtr msg);
  void on_override(const std_msgs::msg::Bool::ConstSharedPtr msg);

  rclcpp::Subscription<CandidateTrajectories>::SharedPtr subs_trajectories_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subs_override_;

  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;

  std::mutex mutex_;

  bool override_enabled_{false};
};

}  // namespace autoware::trajectory_dummy_gate

#endif  // TRAJECTORY_DUMMY_GATE_NODE_HPP_
