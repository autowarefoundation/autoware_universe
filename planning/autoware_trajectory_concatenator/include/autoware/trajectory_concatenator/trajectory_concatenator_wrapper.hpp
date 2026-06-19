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

#ifndef AUTOWARE__TRAJECTORY_CONCATENATOR__TRAJECTORY_CONCATENATOR_WRAPPER_HPP_
#define AUTOWARE__TRAJECTORY_CONCATENATOR__TRAJECTORY_CONCATENATOR_WRAPPER_HPP_

#include <autoware/trajectory_concatenator/detail/trajectory_concatenator.hpp>
#include <autoware_trajectory_concatenator/autoware_trajectory_concatenator_param.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>

#include <memory>
#include <string>

namespace autoware::trajectory_concatenator
{
/**
 * @brief Adapter for TrajectoryConcatenator: handles parameter updates and thread safety.
 */
class TrajectoryConcatenatorWrapper
{
public:
  /**
   * @brief Constructs the wrapper and initialises the concatenator with declared parameters.
   * @param node Node used for parameter declaration and logging.
   * @param node_parameters_interface Parameter interface for declaring and reading parameters.
   */
  TrajectoryConcatenatorWrapper(
    rclcpp::Node & node,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface)
  : node_ptr_(&node),
    logger_(node.get_logger().get_child(interface_name_)),
    concatenator_params_listener_(node_parameters_interface),
    concatenator_params_(concatenator_params_listener_.get_params()),
    concatenator_ptr_(std::make_unique<TrajectoryConcatenator>(concatenator_params_))
  {
  }

  /** @brief Reloads parameters from the parameter server if they have changed. */
  void update_parameters()
  {
    std::lock_guard<std::mutex> lock(concatenator_mutex_);
    if (concatenator_params_listener_.is_old(concatenator_params_)) {
      concatenator_params_ = concatenator_params_listener_.get_params();
      concatenator_ptr_->update_parameters(concatenator_params_);
      RCLCPP_INFO(logger_, "Trajectory Concatenator parameters are updated.");
    }
  }

  /**
   * @brief Adds a set of candidate trajectories to the buffer.
   * @param candidate_trajectories Candidate trajectories from a single generator.
   */
  void add_candidate(
    const autoware_internal_planning_msgs::msg::CandidateTrajectories & candidate_trajectories)
  {
    std::lock_guard<std::mutex> lock(concatenator_mutex_);
    concatenator_ptr_->add_candidate(candidate_trajectories);
  }

  /** @brief Returns the merged set of all non-stale generator trajectories. */
  [[nodiscard]] autoware_internal_planning_msgs::msg::CandidateTrajectories get_concatenated()
  {
    update_parameters();
    const auto time_now = node_ptr_->get_clock()->now();

    std::lock_guard<std::mutex> lock(concatenator_mutex_);
    return concatenator_ptr_->get_concatenated(time_now);
  };

private:
  std::string interface_name_{"trajectory_concatenator"};
  rclcpp::Node * node_ptr_{nullptr};
  rclcpp::Logger logger_;
  concatenator::ParamListener concatenator_params_listener_;
  concatenator::Params concatenator_params_;
  std::unique_ptr<trajectory_concatenator::TrajectoryConcatenator> concatenator_ptr_;
  std::mutex concatenator_mutex_;
};

}  // namespace autoware::trajectory_concatenator

#endif  // AUTOWARE__TRAJECTORY_CONCATENATOR__TRAJECTORY_CONCATENATOR_WRAPPER_HPP_
