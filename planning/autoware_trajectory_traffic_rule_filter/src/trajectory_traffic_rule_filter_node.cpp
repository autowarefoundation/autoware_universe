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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/detail/path_point_with_lane_id__struct.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/Forward.h>

#include <algorithm>
#include <ctime>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace
{
// for error diagnostic. Will be removed once node is combined.
std::unordered_map<std::string, std::string> get_generator_uuid_to_name_map(
  const autoware_internal_planning_msgs::msg::CandidateTrajectories & candidate_trajectories)
{
  std::unordered_map<std::string, std::string> uuid_to_name;
  uuid_to_name.reserve(candidate_trajectories.generator_info.size());
  for (const auto & info : candidate_trajectories.generator_info) {
    uuid_to_name[autoware_utils_uuid::to_hex_string(info.generator_id)] = info.generator_name.data;
  }
  return uuid_to_name;
}

bool has_trajectory_from_generator(
  const std::unordered_map<std::string, std::string> & uuid_to_generator_name_map,
  const autoware_internal_planning_msgs::msg::CandidateTrajectories & trajectories,
  const std::string & generator_name_prefix)
{
  return std::any_of(
    trajectories.candidate_trajectories.cbegin(), trajectories.candidate_trajectories.cend(),
    [&](const autoware_internal_planning_msgs::msg::CandidateTrajectory & trajectory) {
      const auto generator_id_str = autoware_utils_uuid::to_hex_string(trajectory.generator_id);
      const auto generator_name_it = uuid_to_generator_name_map.find(generator_id_str);
      return generator_name_it != uuid_to_generator_name_map.end() &&
             generator_name_it->second.rfind(generator_name_prefix, 0) == 0;
    });
}
}  // namespace

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

  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
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

  constexpr auto log_throttle_ms = 5000;
  if (!lanelet_map_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms, "waiting for lanelet_map");
    return;
  }

  // Get latest traffic light data
  const auto traffic_lights = sub_traffic_lights_.take_data();
  if (traffic_lights) {
    for (auto & plugin : plugins_) {
      plugin->set_traffic_lights(traffic_lights);
    }
  }

  diagnostics_interface_.clear();
  auto filtered_msg = std::make_shared<CandidateTrajectories>();

  for (const auto & trajectory : msg->candidate_trajectories) {
    bool is_feasible = true;
    for (const auto & plugin : plugins_) {
      if (const auto res = plugin->is_feasible(trajectory.points); !res) {
        is_feasible = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), log_throttle_ms, "Not feasible: %s", res.error().c_str());
        diagnostics_interface_.add_key_value(plugin->get_name(), res.error());
      }
    }
    if (is_feasible) {
      filtered_msg->candidate_trajectories.push_back(trajectory);
    }
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

  update_diagnostic(*filtered_msg);
  pub_trajectories_->publish(*filtered_msg);
}

void TrajectoryTrafficRuleFilter::update_diagnostic(
  const CandidateTrajectories & filtered_trajectories)
{
  const auto uuid_to_name_map = get_generator_uuid_to_name_map(filtered_trajectories);
  if (filtered_trajectories.candidate_trajectories.empty()) {
    diagnostics_interface_.update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No feasible trajectories found");
  } else if (!has_trajectory_from_generator(uuid_to_name_map, filtered_trajectories, "Diffusion")) {
    diagnostics_interface_.update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "All diffusion planner trajectories are infeasible");
  } else {
    diagnostics_interface_.update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
  }

  diagnostics_interface_.publish(get_clock()->now());
}

void TrajectoryTrafficRuleFilter::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));

  auto routing_graph_and_traffic_rules =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr_);

  routing_graph_ptr_ =
    autoware::experimental::lanelet2_utils::remove_const(routing_graph_and_traffic_rules.first);
  traffic_rules_ptr_ = routing_graph_and_traffic_rules.second;

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
