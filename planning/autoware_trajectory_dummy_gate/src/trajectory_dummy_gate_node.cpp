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

#include "trajectory_dummy_gate_node.hpp"

#include <rclcpp/logger.hpp>

#include <functional>
#include <mutex>

namespace autoware::trajectory_dummy_gate
{

TrajectoryDummyGateNode::TrajectoryDummyGateNode(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_dummy_gate_node", node_options},
  subs_trajectories_{this->create_subscription<CandidateTrajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryDummyGateNode::on_trajectories, this, std::placeholders::_1))},
  subs_override_{this->create_subscription<std_msgs::msg::Bool>(
    "~/input/override", 1,
    std::bind(&TrajectoryDummyGateNode::on_override, this, std::placeholders::_1))},
  pub_trajectories_{this->create_publisher<CandidateTrajectories>("~/output/trajectories", 1)}
{
}

void TrajectoryDummyGateNode::on_trajectories(const CandidateTrajectories::ConstSharedPtr msg)
{
  bool blocked = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    blocked = override_enabled_;
  }

  if (!blocked) {
    pub_trajectories_->publish(*msg);
    return;
  }

  // Override is ON: block the DiffusionPlanner path. We cannot simply stop publishing, because the
  // downstream concatenator buffers per generator_id and (in this configuration) never time-expires
  // entries, so a stale diffusion-planner trajectory would stay latched in its output. Instead we
  // keep publishing the same generator_info with the candidate_trajectories cleared, which
  // overwrites the concatenator's buffered entry with an empty set so only the backup planner
  // survives downstream.
  auto blocked_msg = *msg;
  blocked_msg.candidate_trajectories.clear();
  pub_trajectories_->publish(blocked_msg);
}

void TrajectoryDummyGateNode::on_override(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (override_enabled_ != msg->data) {
    RCLCPP_INFO(
      get_logger(), "[DummyGate] Override: %s",
      msg->data ? "ON (DiffusionPlanner blocked)" : "OFF (normal)");
    override_enabled_ = msg->data;
  }
}

}  // namespace autoware::trajectory_dummy_gate

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_dummy_gate::TrajectoryDummyGateNode)
