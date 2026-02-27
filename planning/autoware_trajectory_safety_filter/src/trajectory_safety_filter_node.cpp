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

#include "autoware/trajectory_safety_filter/trajectory_safety_filter_node.hpp"

#include "autoware/trajectory_safety_filter/filter_context.hpp"
#include "autoware/trajectory_safety_filter/safety_filter_interface.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/msg/detail/candidate_trajectory__struct.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

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

namespace autoware::trajectory_safety_filter
{

TrajectorySafetyFilter::TrajectorySafetyFilter(const rclcpp::NodeOptions & options)
: Node{"trajectory_safety_filter_node", options},
  listener_{std::make_unique<safety_filter::ParamListener>(get_node_parameters_interface())},
  plugin_loader_(
    "autoware_trajectory_safety_filter",
    "autoware::trajectory_safety_filter::plugin::SafetyFilterInterface"),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  const auto filters = listener_->get_params().filter_names;
  for (const auto & filter : filters) {
    load_metric(filter);
  }

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectorySafetyFilter::map_callback, this, std::placeholders::_1));

  sub_trajectories_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectorySafetyFilter::process, this, std::placeholders::_1));

  pub_trajectories_ = create_publisher<CandidateTrajectories>("~/output/trajectories", 1);

  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/feasible_trajectory_filter", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&TrajectorySafetyFilter::on_parameter, this, std::placeholders::_1));
}

void TrajectorySafetyFilter::process(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  // Prepare context for filters
  FilterContext context;

  context.odometry = sub_odometry_.take_data();
  if (!context.odometry) {
    return;
  }

  context.predicted_objects = sub_objects_.take_data();
  if (!context.predicted_objects) {
    return;
  }

  context.acceleration = sub_acceleration_.take_data();
  if (!context.acceleration) {
    return;
  }

  context.lanelet_map = lanelet_map_ptr_;
  if (!context.lanelet_map) {
    return;
  }

  diagnostics_interface_.clear();

  // Create output message for filtered trajectories
  auto filtered_msg = std::make_unique<CandidateTrajectories>();

  // Process and filter trajectories
  for (const auto & trajectory : msg->candidate_trajectories) {
    if (!validate_trajectory_basics(trajectory)) {
      continue;
    }

    // Apply each filter to the trajectory
    bool is_feasible = true;
    for (const auto & plugin : plugins_) {
      if (const auto res = plugin->is_feasible(trajectory.points, context); !res) {
        is_feasible = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Not feasible: %s", res.error().c_str());
        diagnostics_interface_.add_key_value(plugin->get_name(), res.error());
      }
    }

    if (is_feasible) filtered_msg->candidate_trajectories.push_back(trajectory);
  }

  // Also filter generator_info to match kept trajectories
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

void TrajectorySafetyFilter::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
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

    plugin->set_vehicle_info(vehicle_info_);
    plugin->set_parameters(*this);

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

void TrajectorySafetyFilter::update_diagnostic(const CandidateTrajectories & filtered_trajectories)
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

  diagnostics_interface_.publish(this->get_clock()->now());
}

bool TrajectorySafetyFilter::validate_trajectory_basics(
  const CandidateTrajectory & trajectory) const
{
  // Check minimum trajectory length
  if (trajectory.points.size() < 2) {
    return false;
  }

  // Check all points have finite values
  return std::all_of(
    trajectory.points.begin(), trajectory.points.end(),
    [this](const auto & point) { return check_finite(point); });
}

bool TrajectorySafetyFilter::check_finite(const TrajectoryPoint & point) const
{
  const auto & p = point.pose.position;
  const auto & o = point.pose.orientation;

  using std::isfinite;
  const bool p_result = isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
  const bool quat_result = isfinite(o.x) && isfinite(o.y) && isfinite(o.z) && isfinite(o.w);
  const bool v_result = isfinite(point.longitudinal_velocity_mps);
  const bool w_result = isfinite(point.heading_rate_rps);
  const bool a_result = isfinite(point.acceleration_mps2);

  return p_result && quat_result && v_result && w_result && a_result;
}

rcl_interfaces::msg::SetParametersResult TrajectorySafetyFilter::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    // Broadcast the changed parameters to all loaded plugins
    for (const auto & plugin : plugins_) {
      plugin->update_parameters(parameters);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    // Cleanly reject the parameter change if any plugin detects a type mismatch
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}
}  // namespace autoware::trajectory_safety_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_safety_filter::TrajectorySafetyFilter)
