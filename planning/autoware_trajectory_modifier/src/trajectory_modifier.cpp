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

#include "autoware/trajectory_modifier/trajectory_modifier.hpp"

#include <autoware_utils/ros/update_param.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>

namespace autoware::trajectory_modifier
{

TrajectoryModifier::TrajectoryModifier(const rclcpp::NodeOptions & options)
: Node("trajectory_modifier", options)
{
  set_up_params();

  trajectories_sub_ = create_subscription<CandidateTrajectories>(
    "~/input/candidate_trajectories", 1,
    std::bind(&TrajectoryModifier::on_traj, this, std::placeholders::_1));

  trajectories_pub_ = create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);

  debug_processing_time_detail_pub_ =
    create_publisher<autoware_utils::ProcessingTimeDetail>("~/debug/processing_time_detail", 1);

  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>();

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&TrajectoryModifier::on_parameter, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "TrajectoryModifier initialized");
}

void TrajectoryModifier::on_traj(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!initialized_modifiers_) {
    initialize_modifiers();
  }

  current_odometry_ptr_ = sub_current_odometry_.take_data();
  current_acceleration_ptr_ = sub_current_acceleration_.take_data();

  if (!current_odometry_ptr_ || !current_acceleration_ptr_) {
    RCLCPP_WARN(get_logger(), "Required input data not available");
    return;
  }

  params_.current_odometry = *current_odometry_ptr_;
  params_.current_acceleration = *current_acceleration_ptr_;

  CandidateTrajectories output_trajectories = *msg;

  for (auto & trajectory : output_trajectories.candidate_trajectories) {
    for (auto & modifier : modifier_plugins_) {
      modifier->modify_trajectory(trajectory.points, params_);
    }
  }

  trajectories_pub_->publish(output_trajectories);
}

void TrajectoryModifier::set_up_params()
{
}

void TrajectoryModifier::initialize_modifiers()
{
  initialized_modifiers_ = true;
  RCLCPP_INFO(get_logger(), "Trajectory modifier plugins initialized");
}

void TrajectoryModifier::reset_previous_data()
{
  prev_modified_traj_points_ptr_.reset();
}

rcl_interfaces::msg::SetParametersResult TrajectoryModifier::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;
  auto params = params_;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_modifier

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_modifier::TrajectoryModifier)
