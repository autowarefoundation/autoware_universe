// Copyright 2024 TIER IV, Inc.
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

#include "autoware/trajectory_processor/trajectory_optimizer.hpp"

#include "autoware/trajectory_processor/utils.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer
{

TrajectoryOptimizer::TrajectoryOptimizer(const rclcpp::NodeOptions & options)
: Node("trajectory_optimizer", options),
  plugin_loader_(
    std::make_unique<pluginlib::ClassLoader<plugin::TrajectoryOptimizerPluginBase>>(
      "autoware_trajectory_processor",
      "autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase"))
{
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  debug_processing_time_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  param_listener_ = std::make_unique<trajectory_optimizer_node_params::ParamListener>(
    get_node_parameters_interface());
  params_ = param_listener_->get_params();

  initialize_optimizers();

  trajectories_sub_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryOptimizer::on_traj, this, std::placeholders::_1));
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  trajectories_pub_ = create_publisher<CandidateTrajectories>("~/output/trajectories", 1);
}

void TrajectoryOptimizer::initialize_optimizers()
{
  if (initialized_optimizers_) {
    return;
  }

  // Get plugin names from parameter
  const auto plugin_names = params_.plugin_names;

  // Load each plugin in order
  for (const auto & plugin_name : plugin_names) {
    load_plugin(plugin_name);
  }
  initialized_optimizers_ = true;
}

void TrajectoryOptimizer::load_plugin(const std::string & plugin_name)
{
  try {
    auto plugin = plugin_loader_->createSharedInstance(plugin_name);

    // Check if plugin is already loaded
    for (const auto & p : plugins_) {
      if (plugin->get_name() == p->get_name()) {
        RCLCPP_WARN_STREAM(get_logger(), "Plugin '" << plugin_name << "' is already loaded.");
        return;
      }
    }

    // Initialize plugin with node context
    plugin->initialize(plugin_name, this, time_keeper_, params_);

    plugins_.push_back(plugin);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded plugin: " << plugin_name);
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to load plugin '" << plugin_name << "': " << e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Unexpected error loading plugin '" << plugin_name << "': " << e.what());
  }
}

void TrajectoryOptimizer::update_params()
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

void TrajectoryOptimizer::publish_processing_time_ms(const double processing_time_ms)
{
  autoware_internal_debug_msgs::msg::Float64Stamped msg;
  msg.stamp = get_clock()->now();
  msg.data = processing_time_ms;
  debug_processing_time_pub_->publish(msg);
}

void TrajectoryOptimizer::on_traj([[maybe_unused]] const CandidateTrajectories::ConstSharedPtr msg)
{
  stop_watch_ptr_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing_time");
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  initialize_optimizers();

  if (param_listener_->is_old(params_)) {
    update_params();
  }

  current_odometry_ptr_ = sub_current_odometry_.take_data();
  current_acceleration_ptr_ = sub_current_acceleration_.take_data();

  if (!current_odometry_ptr_ || !current_acceleration_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No odometry or acceleration data");
    publish_processing_time_ms(stop_watch_ptr_->toc("processing_time", true));
    return;
  }

  CandidateTrajectories output_trajectories = *msg;
  for (auto & trajectory : output_trajectories.candidate_trajectories) {
    // Create a fresh data instance per trajectory so semantic_speed_tracker is reset each time
    TrajectoryOptimizerData data;
    data.current_odometry = *current_odometry_ptr_;
    data.current_acceleration = *current_acceleration_ptr_;
    // Apply optimizations - plugins execute in order from plugin_names parameter
    for (auto & plugin : plugins_) {
      plugin->optimize_trajectory(trajectory.points, params_, data);
    }

    // Downstream Autoware modules dont properly support trajectories with less than 3 points. So we
    // return a dummy stopped trajectory instead.
    if (trajectory.points.size() < 3) {
      trajectory.points =
        utils::generate_three_point_stopped_trajectory(trajectory.points, data.current_odometry);
    }
  }

  trajectories_pub_->publish(output_trajectories);

  Trajectory output_trajectory;
  output_trajectory.header = output_trajectories.candidate_trajectories.front().header;
  output_trajectory.points = output_trajectories.candidate_trajectories.front().points;

  trajectory_pub_->publish(output_trajectory);

  publish_processing_time_ms(stop_watch_ptr_->toc("processing_time", true));
}

}  // namespace autoware::trajectory_optimizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_optimizer::TrajectoryOptimizer)
