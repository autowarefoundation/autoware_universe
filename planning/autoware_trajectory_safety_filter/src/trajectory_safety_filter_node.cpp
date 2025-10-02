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

#include <autoware_internal_planning_msgs/msg/detail/candidate_trajectory__struct.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

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

  context.lanelet_map = lanelet_map_ptr_;
  if (!context.lanelet_map) {
    return;
  }

  // Create output message for filtered trajectories
  auto filtered_msg = std::make_shared<CandidateTrajectories>();

  // Process and filter trajectories
  for (const auto & trajectory : msg->candidate_trajectories) {
    if (!validate_trajectory_basics(trajectory)) {
      continue;
    }

    // Apply each filter to the trajectory
    bool is_feasible = true;
    for (const auto & plugin : plugins_) {
      if (!plugin->is_feasible(trajectory.points, context)) {
        is_feasible = false;
        break;
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

  pub_trajectories_->publish(*filtered_msg);
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

    // Load parameters from ROS and pass to plugin
    std::unordered_map<std::string, std::any> params;
    const auto all_params = listener_->get_params();

    // Determine parameter prefix based on plugin name
    if (name.find("OutOfLaneFilter") != std::string::npos) {
      params["out_of_lane.time"] = all_params.out_of_lane.time;
      params["out_of_lane.min_value"] = all_params.out_of_lane.min_value;
    } else if (name.find("CollisionFilter") != std::string::npos) {
      params["collision.time"] = all_params.collision.time;
      params["collision.min_value"] = all_params.collision.min_value;
    }

    plugin->set_vehicle_info(vehicle_info_);
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
}  // namespace autoware::trajectory_safety_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_safety_filter::TrajectorySafetyFilter)
