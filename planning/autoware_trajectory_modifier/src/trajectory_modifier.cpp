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
#include <string>
#include <vector>

namespace autoware::trajectory_modifier
{

TrajectoryModifier::TrajectoryModifier(const rclcpp::NodeOptions & options)
: Node{"trajectory_modifier", options},
  param_listener_{
    std::make_unique<trajectory_modifier_params::ParamListener>(get_node_parameters_interface())},
  plugin_loader_(
    "autoware_trajectory_modifier",
    "autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase")
{
  trajectories_sub_ = create_subscription<CandidateTrajectories>(
    "~/input/candidate_trajectories", 1,
    std::bind(&TrajectoryModifier::on_traj, this, std::placeholders::_1));
  trajectories_pub_ = create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);

  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail", 1);

  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();

  params_ = param_listener_->get_params();

  for (const auto & name : this->declare_parameter<std::vector<std::string>>("launch_modules")) {
    if (name.empty()) continue;
    load_plugin(name);
  }

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

  if (param_listener_->is_old(params_)) {
    update_params();
  }

  for (auto & trajectory : output_trajectories.candidate_trajectories) {
    for (auto & modifier : plugins_) {
      modifier->modify_trajectory(trajectory.points, data_);
      modifier->publish_planning_factor();
    }
  }

  trajectories_pub_->publish(output_trajectories);
}

void TrajectoryModifier::load_plugin(const std::string & name)
{
  // Check if the plugin is already loaded.
  if (plugin_loader_.isClassLoaded(name)) {
    RCLCPP_WARN(this->get_logger(), "The plugin '%s' is already loaded.", name.c_str());
    return;
  }
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);
    plugin->initialize(name, this, time_keeper_, params_);
    // register
    plugins_.push_back(plugin);
    RCLCPP_DEBUG(this->get_logger(), "The plugin '%s' has been loaded", name.c_str());
    initialized_modifiers_ = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "The plugin '%s' is not available", name.c_str());
  }
}

void TrajectoryModifier::unload_plugin(const std::string & name)
{
  auto it = std::remove_if(plugins_.begin(), plugins_.end(), [&](const auto plugin) {
    return plugin->get_name() == name;
  });

  if (it == plugins_.end()) {
    RCLCPP_WARN(
      this->get_logger(), "The plugin '%s' is not in the registered modules", name.c_str());
  } else {
    plugins_.erase(it, plugins_.end());
    RCLCPP_INFO(this->get_logger(), "The scene plugin '%s' has been unloaded", name.c_str());
  }
}

void TrajectoryModifier::update_params()
{
  try {
    params_ = param_listener_->get_params();

    for (auto & plugin : plugins_) {
      plugin->update_params(params_);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Failed to update parameters: %s", e.what());
  }
}

}  // namespace autoware::trajectory_modifier

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_modifier::TrajectoryModifier)
