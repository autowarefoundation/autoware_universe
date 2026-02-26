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

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      this, "trajectory_adapter");

  param_listener_ =
    std::make_unique<::trajectory_adapter::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();
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

  publish_planning_factor(trajectory);
}

void TrajectoryAdapterNode::publish_planning_factor(const Trajectory & trajectory)
{
  const auto & points = trajectory.points;

  const auto & p = params_;
  const double stop_keep_duration_threshold = p.stop_keep_duration_threshold;
  const double stop_velocity_threshold = p.stop_velocity_threshold;
  const double slowdown_accel_threshold = p.slowdown_accel_threshold;

  std::optional<size_t> slowdown_start_idx;
  std::optional<size_t> slowdown_end_idx;
  std::optional<size_t> stop_idx;
  rclcpp::Duration stop_start_time(0, 0);
  bool is_valid_stop = true;

  // find index
  for (size_t i = 0; i < points.size(); ++i) {
    // for stop
    if (!stop_idx && points[i].longitudinal_velocity_mps <= stop_velocity_threshold) {
      stop_idx = i;
      stop_start_time = rclcpp::Duration(points[i].time_from_start);
    }
    if (stop_idx) {
      if (
        (rclcpp::Duration(points[i].time_from_start) - stop_start_time).seconds() <
        stop_keep_duration_threshold) {
        if (points[i].longitudinal_velocity_mps > stop_velocity_threshold) is_valid_stop = false;
      }
    }

    // for slowdown
    if (points[i].acceleration_mps2 < slowdown_accel_threshold) {
      if (!slowdown_start_idx) slowdown_start_idx = i;
    } else if (slowdown_start_idx && !slowdown_end_idx) {
      slowdown_end_idx = i;
    }
  }

  const auto start_pos = points[0].pose;

  // stop planning factor
  if (stop_idx && is_valid_stop) {
    planning_factor_interface_->add(
      points, start_pos, points[*stop_idx].pose, PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{});
  }

  // slowdown planning factor
  if (slowdown_start_idx && slowdown_end_idx) {
    planning_factor_interface_->add(
      points, start_pos, points[*slowdown_start_idx].pose, points[*slowdown_end_idx].pose,
      PlanningFactor::SLOW_DOWN, autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true,
      static_cast<double>(points[*slowdown_start_idx].longitudinal_velocity_mps),
      static_cast<double>(points[*slowdown_end_idx].longitudinal_velocity_mps));
  }

  planning_factor_interface_->publish();
}

}  // namespace autoware::trajectory_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_adapter::TrajectoryAdapterNode)
