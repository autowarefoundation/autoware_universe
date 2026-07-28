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

#ifndef AUTOWARE__TRAJECTORY_RANKER__TRAJECTORY_RANKER_WRAPPER_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__TRAJECTORY_RANKER_WRAPPER_HPP_

#include "autoware/trajectory_ranker/trajectory_ranker.hpp"

#include <autoware_trajectory_ranker/autoware_trajectory_ranker_param.hpp>
#include <autoware_trajectory_validator/msg/validation_report.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_ranker
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories;
using autoware_trajectory_validator::msg::ValidationReport;

class TrajectoryRankerWrapper
{
public:
  explicit TrajectoryRankerWrapper(
    rclcpp::Node & node,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
    vehicle_info_utils::VehicleInfo vehicle_info,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper);

  ScoredCandidateTrajectories rank_trajectories(
    const RankerInputTrajectories & input_trajectories, const RankerContext & context);

private:
  void update_parameters();
  void update_last_best_trajectory_info(const ScoredTrajectory & best_trajectory_info);

  rclcpp::Node * node_ptr_{nullptr};
  std::string interface_name_{"trajectory_ranker"};
  rclcpp::Logger logger_;
  std::shared_ptr<vehicle_info_utils::VehicleInfo> vehicle_info_;
  std::unique_ptr<TrajectoryRanker> ranker_ptr_;
  std::shared_ptr<Evaluator> evaluator_;

  std::optional<ScoredTrajectory> last_best_trajectory_info_;

  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
  std::unique_ptr<trajectory_ranker_params::ParamListener> param_listener_;
  trajectory_ranker_params::Params params_;
};

}  // namespace autoware::trajectory_ranker

#endif  // AUTOWARE__TRAJECTORY_RANKER__TRAJECTORY_RANKER_WRAPPER_HPP_
