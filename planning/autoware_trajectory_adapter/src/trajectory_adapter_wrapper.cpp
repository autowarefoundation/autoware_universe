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

#include "autoware/trajectory_adapter/trajectory_adapter_wrapper.hpp"

#include <memory>
#include <utility>

namespace autoware::trajectory_adapter
{

TrajectoryAdapterWrapper::TrajectoryAdapterWrapper(
  autoware::agnocast_wrapper::Node & node,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: node_ptr_(&node),
  logger_(node_ptr_->get_logger().get_child(interface_name_)),
  adapter_ptr_(std::make_unique<TrajectoryAdapter>()),
  time_keeper_(std::move(time_keeper))
{
  if (!time_keeper_) {
    throw std::runtime_error("TimeKeeper is required for TrajectoryAdapterWrapper");
  }

  debug_latency_pub_ =
    node_ptr_->create_publisher<Float64Stamped>("~/debug/planning_component_latency_s", 1);
}

std::optional<TrajectoryAdapterResult> TrajectoryAdapterWrapper::get_trajectory(
  const ScoredCandidateTrajectories & scored_trajectories)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto result = adapter_ptr_->process(scored_trajectories);
  if (!result) {
    RCLCPP_ERROR(logger_, "Failed to get adapted trajectory: %s", result.error().c_str());
    return std::nullopt;
  }

  const auto & result_value = result.value();

  RCLCPP_DEBUG(
    logger_, "best generator: %s score: %f", result_value.best_generator_name.c_str(),
    result_value.best_score);

  autoware_internal_debug_msgs::msg::Float64Stamped latency_msg;
  latency_msg.stamp = node_ptr_->now();
  latency_msg.data = (node_ptr_->now() - result_value.trajectory.header.stamp).seconds();
  debug_latency_pub_->publish(latency_msg);

  return result_value;
}

}  // namespace autoware::trajectory_adapter
