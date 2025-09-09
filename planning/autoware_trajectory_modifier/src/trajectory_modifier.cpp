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
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <vector>

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

  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail", 1);

  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&TrajectoryModifier::on_parameter, this, std::placeholders::_1));

  initialize_modifiers();

  RCLCPP_INFO(get_logger(), "TrajectoryModifier initialized");
}

void TrajectoryModifier::on_traj(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!initialized_modifiers_) {
    throw std::runtime_error("Modifiers not initialized");
  }

  current_odometry_ptr_ = sub_current_odometry_.take_data();
  current_acceleration_ptr_ = sub_current_acceleration_.take_data();

  if (!current_odometry_ptr_ || !current_acceleration_ptr_) {
    return;
  }

  data_.current_odometry = *current_odometry_ptr_;
  data_.current_acceleration = *current_acceleration_ptr_;

  CandidateTrajectories output_trajectories = *msg;

  for (auto & trajectory : output_trajectories.candidate_trajectories) {
    for (auto & modifier : modifier_plugins_) {
      modifier->modify_trajectory(trajectory.points, params_, data_);
    }
  }

  trajectories_pub_->publish(output_trajectories);
}

void TrajectoryModifier::set_up_params()
{
  // Declare parameter with default value
  rcl_interfaces::msg::ParameterDescriptor use_stop_point_fixer_desc;
  use_stop_point_fixer_desc.description = "Enable the stop point fixer modifier plugin";
  params_.use_stop_point_fixer =
    this->declare_parameter<bool>("use_stop_point_fixer", true, use_stop_point_fixer_desc);
}

void TrajectoryModifier::initialize_modifiers()
{
  stop_point_fixer_ptr_ =
    std::make_shared<plugin::StopPointFixer>("stop_point_fixer", this, time_keeper_, params_);
  modifier_plugins_.push_back(stop_point_fixer_ptr_);
  RCLCPP_INFO(get_logger(), "StopPointFixer plugin initialized");

  initialized_modifiers_ = true;
  RCLCPP_INFO(
    get_logger(), "Trajectory modifier plugins initialized: %zu plugins", modifier_plugins_.size());
}

rcl_interfaces::msg::SetParametersResult TrajectoryModifier::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    update_param<bool>(parameters, "use_stop_point_fixer", params_.use_stop_point_fixer);

    for (auto & modifier : modifier_plugins_) {
      auto plugin_result = modifier->on_parameter(parameters);
      if (!plugin_result.successful) {
        result.successful = false;
        result.reason = plugin_result.reason;
      }
    }
  } catch (const std::exception & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

}  // namespace autoware::trajectory_modifier

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_modifier::TrajectoryModifier)
