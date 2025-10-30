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

#include "autoware/trajectory_optimizer/trajectory_optimizer.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer
{

TrajectoryOptimizer::TrajectoryOptimizer(const rclcpp::NodeOptions & options)
: Node("trajectory_optimizer", options),
  plugin_loader_(
    std::make_unique<pluginlib::ClassLoader<plugin::TrajectoryOptimizerPluginBase>>(
      "autoware_trajectory_optimizer",
      "autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase"))
{
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  set_up_params();
  initialize_optimizers();

  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&TrajectoryOptimizer::on_parameter, this, std::placeholders::_1));

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
  const auto plugin_names = get_parameter("plugin_names").as_string_array();

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
    plugin->initialize(plugin_name, this, time_keeper_);

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

rcl_interfaces::msg::SetParametersResult TrajectoryOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;
  auto params = params_;

  update_param<bool>(
    parameters, "use_akima_spline_interpolation", params.use_akima_spline_interpolation);
  update_param<bool>(parameters, "use_eb_smoother", params.use_eb_smoother);
  update_param<bool>(parameters, "use_qp_smoother", params.use_qp_smoother);
  update_param<bool>(parameters, "fix_invalid_points", params.fix_invalid_points);
  update_param<bool>(parameters, "optimize_velocity", params.optimize_velocity);
  update_param<bool>(parameters, "extend_trajectory_backward", params.extend_trajectory_backward);

  params_ = params;

  // Forward parameter updates to all loaded plugins
  for (auto & plugin : plugins_) {
    plugin->on_parameter(parameters);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void TrajectoryOptimizer::set_up_params()
{
  using autoware_utils_rclcpp::get_or_declare_parameter;

  // Declare plugin_names parameter with default order
  const std::vector<std::string> default_plugins = {
    "autoware::trajectory_optimizer::plugin::TrajectoryPointFixer",
    "autoware::trajectory_optimizer::plugin::TrajectoryQPSmoother",
    "autoware::trajectory_optimizer::plugin::TrajectoryEBSmootherOptimizer",
    "autoware::trajectory_optimizer::plugin::TrajectorySplineSmoother",
    "autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer",
    "autoware::trajectory_optimizer::plugin::TrajectoryExtender",
    "autoware::trajectory_optimizer::plugin::TrajectoryPointFixer"};
  declare_parameter("plugin_names", default_plugins);

  params_.use_akima_spline_interpolation =
    get_or_declare_parameter<bool>(*this, "use_akima_spline_interpolation");
  params_.use_eb_smoother = get_or_declare_parameter<bool>(*this, "use_eb_smoother");
  params_.use_qp_smoother = get_or_declare_parameter<bool>(*this, "use_qp_smoother");
  params_.fix_invalid_points = get_or_declare_parameter<bool>(*this, "fix_invalid_points");
  params_.optimize_velocity = get_or_declare_parameter<bool>(*this, "optimize_velocity");
  params_.extend_trajectory_backward =
    get_or_declare_parameter<bool>(*this, "extend_trajectory_backward");
}

void TrajectoryOptimizer::on_traj([[maybe_unused]] const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  initialize_optimizers();

  current_odometry_ptr_ = sub_current_odometry_.take_data();
  current_acceleration_ptr_ = sub_current_acceleration_.take_data();

  if (!current_odometry_ptr_ || !current_acceleration_ptr_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "No odometry or acceleration data");
    return;
  }

  // Create runtime data struct
  TrajectoryOptimizerData data;
  data.current_odometry = *current_odometry_ptr_;
  data.current_acceleration = *current_acceleration_ptr_;

  CandidateTrajectories output_trajectories = *msg;
  for (auto & trajectory : output_trajectories.candidate_trajectories) {
    // Apply optimizations - plugins execute in order from plugin_names parameter
    for (auto & plugin : plugins_) {
      plugin->optimize_trajectory(trajectory.points, params_, data);
    }
    motion_utils::calculate_time_from_start(
      trajectory.points, current_odometry_ptr_->pose.pose.position);
  }

  trajectories_pub_->publish(output_trajectories);

  Trajectory output_trajectory;
  output_trajectory.header = output_trajectories.candidate_trajectories.front().header;
  output_trajectory.points = output_trajectories.candidate_trajectories.front().points;

  trajectory_pub_->publish(output_trajectory);
}

}  // namespace autoware::trajectory_optimizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_optimizer::TrajectoryOptimizer)
