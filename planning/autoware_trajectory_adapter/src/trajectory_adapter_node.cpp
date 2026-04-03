// Copyright 2025 TIER IV, Inc.
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

#include "trajectory_adapter_node.hpp"

#include <memory>

namespace autoware::trajectory_adapter
{

TrajectoryAdapterNode::TrajectoryAdapterNode(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_adapter_node", node_options},
  sub_trajectories_{this->create_subscription<ScoredCandidateTrajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryAdapterNode::process, this, std::placeholders::_1))},
  pub_trajectory_{this->create_publisher<Trajectory>("~/output/trajectory", 1)}
{
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/trajectory_adapter", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);
}

void TrajectoryAdapterNode::process(const ScoredCandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (msg->scored_candidate_trajectories.empty()) {
    return;
  }

  const auto trajectory_itr = std::max_element(
    msg->scored_candidate_trajectories.begin(), msg->scored_candidate_trajectories.end(),
    [](const auto & a, const auto & b) { return a.score < b.score; });

  const auto best_generator = [&msg](const auto & uuid) {
    const auto generator_itr = std::find_if(
      msg->generator_info.begin(), msg->generator_info.end(),
      [&uuid](const auto & info) { return info.generator_id == uuid; });
    return generator_itr == msg->generator_info.end() ? "NOT FOUND"
                                                      : generator_itr->generator_name.data;
  };

  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "best generator:" << best_generator(trajectory_itr->candidate_trajectory.generator_id)
                      << " score:" << trajectory_itr->score);

  const auto trajectory = autoware_planning_msgs::build<Trajectory>()
                            .header(trajectory_itr->candidate_trajectory.header)
                            .points(trajectory_itr->candidate_trajectory.points);

  pub_trajectory_->publish(trajectory);
}

}  // namespace autoware::trajectory_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_adapter::TrajectoryAdapterNode)
