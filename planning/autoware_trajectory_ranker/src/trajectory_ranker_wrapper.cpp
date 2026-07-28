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

#include "autoware/trajectory_ranker/trajectory_ranker_wrapper.hpp"

#include <magic_enum.hpp>

#include <memory>
#include <string>
#include <utility>

namespace autoware::trajectory_ranker
{

TrajectoryRankerWrapper::TrajectoryRankerWrapper(
  rclcpp::Node & node,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
  VehicleInfo vehicle_info, std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: node_ptr_(&node),
  logger_(node.get_logger().get_child(interface_name_)),
  vehicle_info_(std::make_shared<VehicleInfo>(vehicle_info)),
  time_keeper_(std::move(time_keeper)),
  param_listener_(
    std::make_unique<trajectory_ranker_params::ParamListener>(node_parameters_interface))
{
  if (!time_keeper_) {
    throw std::runtime_error("TimeKeeper is required for TrajectoryRankerWrapper");
  }

  params_ = param_listener_->get_params();

  evaluator_ =
    std::make_shared<Evaluator>(vehicle_info_, node.get_logger(), params_.evaluation, node_ptr_);

  ranker_ptr_ = std::make_unique<TrajectoryRanker>(evaluator_, params_);
}

void TrajectoryRankerWrapper::update_parameters()
{
  if (!param_listener_->is_old(params_)) return;

  params_ = param_listener_->get_params();
  if (ranker_ptr_) ranker_ptr_->update_parameters(params_);
  RCLCPP_INFO(logger_, "Trajectory Ranker parameters are updated.");
}

ScoredCandidateTrajectories TrajectoryRankerWrapper::rank_trajectories(
  const RankerInputTrajectories & input_trajectories, const RankerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  update_parameters();

  auto result = ranker_ptr_->process(input_trajectories, context);
  if (!result) {
    RCLCPP_ERROR(logger_, "Failed to rank trajectories: %s", result.error().c_str());
    return ScoredCandidateTrajectories();
  }

  update_last_best_trajectory_info(result.value().best_trajectory_info);

  return result.value().scored_trajectories;
}

void TrajectoryRankerWrapper::update_last_best_trajectory_info(
  const ScoredTrajectory & best_trajectory_info)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  auto source = std::string(magic_enum::enum_name(best_trajectory_info.input_trajectory.source));
  auto last_source =
    last_best_trajectory_info_
      ? std::string(magic_enum::enum_name(last_best_trajectory_info_->input_trajectory.source))
      : "none";
  bool is_source_changed = last_source != source;
  last_best_trajectory_info_ = best_trajectory_info;

  bool is_low_score = last_best_trajectory_info_->score < 0.5;

  if (is_source_changed) {
    RCLCPP_WARN(
      logger_, "[Ranker] Best trajectory source changed from %s to %s", last_source.c_str(),
      source.c_str());
  }

  if (is_low_score) {
    RCLCPP_WARN(
      logger_, "[Ranker] Best trajectory score is low: %f", last_best_trajectory_info_->score);
  }

  if (!is_source_changed && !is_low_score) return;

  RCLCPP_INFO(
    logger_,
    "[Ranker] Best trajectory info: safety_penalty -> %f, source_penalty -> %f, quality_penalty -> "
    "%f, "
    "score -> %f",
    last_best_trajectory_info_->safety_penalty, last_best_trajectory_info_->source_penalty,
    last_best_trajectory_info_->quality_penalty, last_best_trajectory_info_->score);
}

}  // namespace autoware::trajectory_ranker
