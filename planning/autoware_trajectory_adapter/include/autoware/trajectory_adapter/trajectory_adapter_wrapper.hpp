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

#ifndef AUTOWARE__TRAJECTORY_ADAPTER__TRAJECTORY_ADAPTER_WRAPPER_HPP_
#define AUTOWARE__TRAJECTORY_ADAPTER__TRAJECTORY_ADAPTER_WRAPPER_HPP_

#include "autoware/trajectory_adapter/trajectory_adapter.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>

#include <memory>
#include <optional>
#include <string>

namespace autoware::trajectory_adapter
{
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories;

/**
 * @brief Adapter for TrajectoryAdapter: handles latency debug publishing and processing time
 * tracking.
 */
class TrajectoryAdapterWrapper
{
public:
  /**
   * @brief Constructs the wrapper and initialises debug publishers.
   * @param node Agnocast wrapper node used for publisher creation and logging.
   * @param time_keeper Shared time keeper for processing time tracking.
   */
  TrajectoryAdapterWrapper(
    autoware::agnocast_wrapper::Node & node,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper);

  /**
   * @brief Selects the best trajectory from ranked candidates.
   * @param scored_trajectories Ranked candidate trajectories to adapt.
   * @return The adapted outputs, or std::nullopt if the input is empty.
   */
  [[nodiscard]] std::optional<TrajectoryAdapterResult> get_trajectory(
    const ScoredCandidateTrajectories & scored_trajectories);

private:
  autoware::agnocast_wrapper::Node * node_ptr_{nullptr};
  std::string interface_name_{"trajectory_adapter"};
  rclcpp::Logger logger_;
  std::unique_ptr<TrajectoryAdapter> adapter_ptr_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};

  AUTOWARE_PUBLISHER_PTR(Float64Stamped) debug_latency_pub_;
};

}  // namespace autoware::trajectory_adapter

#endif  // AUTOWARE__TRAJECTORY_ADAPTER__TRAJECTORY_ADAPTER_WRAPPER_HPP_
