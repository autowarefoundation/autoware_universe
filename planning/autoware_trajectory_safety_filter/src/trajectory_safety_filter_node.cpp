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

#include "trajectory_safety_filter_node.hpp"

#include "autoware/trajectory_safety_filter/safety_filter_interface.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::trajectory_safety_filter
{

TrajectorySafetyFilter::TrajectorySafetyFilter(const rclcpp::NodeOptions & options)
: Node{"trajectory_safety_filter_node", options},
  listener_{std::make_unique<safety_filter::ParamListener>(get_node_parameters_interface())},
  plugin_loader_(
    "autoware_trajectory_safety_filter",
    "autoware::trajectory_safety_filter::plugin::SafetyFilterInterface")
{
  const auto filters = listener_->get_params().filter_names;
  for (const auto & filter : filters) {
    load_metric(filter);
  }

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectorySafetyFilter::map_callback, this, std::placeholders::_1));

  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/feasible_trajectory_filter", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);
}

void TrajectorySafetyFilter::process(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  pub_trajectories_->publish(*msg);

  const auto odometry_ptr = sub_odometry_.take_data();
}

void TrajectorySafetyFilter::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
}

void TrajectorySafetyFilter::load_metric(const std::string & name)
{
  try {
    auto plugin = plugin_loader_.createSharedInstance(name);

    for (const auto & p : plugins_) {
      if (plugin->get_name() == p->get_name()) {
        RCLCPP_WARN_STREAM(get_logger(), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    // Initialize the plugin
    plugin->initialize(name);

    // Load parameters from ROS and pass to plugin
    std::unordered_map<std::string, std::any> params;
    const std::string prefix = "safety_filters." + name;

    // Try to get parameters (with defaults)
    params["max_distance_from_trajectory"] =
      this->declare_parameter<double>(prefix + ".max_distance_from_trajectory", 5.0);
    params["check_finite_values"] =
      this->declare_parameter<bool>(prefix + ".check_finite_values", true);
    params["check_trajectory_length"] =
      this->declare_parameter<bool>(prefix + ".check_trajectory_length", true);
    params["min_trajectory_length"] =
      static_cast<size_t>(this->declare_parameter<int>(prefix + ".min_trajectory_length", 2));

    plugin->set_parameters(params);

    plugins_.push_back(plugin);
    RCLCPP_INFO_STREAM(
      get_logger(), "The scene plugin '" << name << "' is loaded and initialized.");
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[safety_filter] createSharedInstance failed for '" << name << "': " << e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "[safety_filter] unexpected exception for '" << name << "': " << e.what());
  }
}

void TrajectorySafetyFilter::unload_metric(const std::string & name)
{
  auto it = std::remove_if(
    plugins_.begin(), plugins_.end(),
    [&](const std::shared_ptr<plugin::SafetyFilterInterface> & plugin) {
      return plugin->get_name() == name;
    });

  if (it == plugins_.end()) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    plugins_.erase(it, plugins_.end());
    RCLCPP_INFO_STREAM(get_logger(), "The scene plugin '" << name << "' is unloaded.");
  }
}

}  // namespace autoware::trajectory_safety_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_safety_filter::TrajectorySafetyFilter)
