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

#include "autoware/trajectory_traffic_rule_filter/trajectory_traffic_rule_filter_node.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/detail/path_point_with_lane_id__struct.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/Forward.h>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>

namespace autoware::trajectory_traffic_rule_filter
{
TrajectoryTrafficRuleFilter::TrajectoryTrafficRuleFilter(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_traffic_rule_filter_node", node_options},
  plugin_loader_(
    "autoware_trajectory_traffic_rule_filter",
    "autoware::trajectory_traffic_rule_filter::plugin::TrafficRuleFilterInterface"),
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()},
  listener_{std::make_unique<traffic_rule_filter::ParamListener>(get_node_parameters_interface())}
{
  const auto filters = listener_->get_params().filter_names;
  for (const auto & filter : filters) {
    load_metric(filter);
  }

  debug_processing_time_detail_pub_ =
    this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectoryTrafficRuleFilter::map_callback, this, std::placeholders::_1));

  sub_trajectories_ = create_subscription<CandidateTrajectories>(
    "~/input/candidate_trajectories", 1,
    std::bind(&TrajectoryTrafficRuleFilter::process, this, std::placeholders::_1));

  pub_trajectories_ = create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);
}

void TrajectoryTrafficRuleFilter::process(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!lanelet_map_ptr_) {
    return;
  }

  // Get latest traffic light data
  const auto traffic_lights = sub_traffic_lights_.take_data();
  if (traffic_lights) {
    for (auto & plugin : plugins_) {
      plugin->set_traffic_lights(traffic_lights);
    }
  }

  auto filtered_msg = std::make_shared<CandidateTrajectories>();

  for (const auto & trajectory : msg->candidate_trajectories) {
    const bool is_feasible = std::all_of(
      plugins_.begin(), plugins_.end(),
      [&](const auto & plugin) { return plugin->is_feasible(trajectory.points); });
    if (is_feasible) filtered_msg->candidate_trajectories.push_back(trajectory);
  }

  std::unordered_set<std::string> kept_generator_ids;
  for (const auto & traj : filtered_msg->candidate_trajectories) {
    std::stringstream ss;
    for (const auto & byte : traj.generator_id.uuid) {
      ss << std::hex << static_cast<int>(byte);
    }
    kept_generator_ids.insert(ss.str());
  }

  for (const auto & gen_info : msg->generator_info) {
    std::stringstream ss;
    for (const auto & byte : gen_info.generator_id.uuid) {
      ss << std::hex << static_cast<int>(byte);
    }
    if (kept_generator_ids.count(ss.str()) > 0) {
      filtered_msg->generator_info.push_back(gen_info);
    }
  }

  pub_trajectories_->publish(*filtered_msg);
}

void TrajectoryTrafficRuleFilter::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  for (const auto & plugin : plugins_) {
    plugin->set_lanelet_map(lanelet_map_ptr_, routing_graph_ptr_, traffic_rules_ptr_);
  }
}

void TrajectoryTrafficRuleFilter::load_metric(const std::string & name)
{
  try {
    auto plugin = plugin_loader_.createSharedInstance(name);

    for (const auto & p : plugins_) {
      if (plugin->get_name() == p->get_name()) {
        RCLCPP_WARN_STREAM(get_logger(), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    plugin->set_vehicle_info(vehicle_info_);
    plugins_.push_back(plugin);

    RCLCPP_INFO_STREAM(
      get_logger(), "The scene plugin '" << name << "' is loaded and initialized.");
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[traffic_rule_filter] createSharedInstance failed for '" << name << "': " << e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[traffic_rule_filter] unexpected exception for '" << name << "': " << e.what());
  }
}

void TrajectoryTrafficRuleFilter::unload_metric(const std::string & name)
{
  auto it = std::remove_if(
    plugins_.begin(), plugins_.end(),
    [&](const std::shared_ptr<plugin::TrafficRuleFilterInterface> & plugin) {
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

}  // namespace autoware::trajectory_traffic_rule_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_traffic_rule_filter::TrajectoryTrafficRuleFilter)
