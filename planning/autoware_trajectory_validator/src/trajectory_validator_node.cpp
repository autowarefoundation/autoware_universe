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

#include "autoware/trajectory_validator/trajectory_validator_node.hpp"

#include "autoware/trajectory_validator/filter_context.hpp"
#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/msg/detail/candidate_trajectory__struct.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
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
}  // namespace

namespace autoware::trajectory_validator
{

TrajectoryValidator::TrajectoryValidator(const rclcpp::NodeOptions & options)
: Node{"trajectory_validator_node", options},
  listener_{get_node_parameters_interface()},
  params_(listener_.get_params()),
  plugin_loader_(
    "autoware_trajectory_validator", "autoware::trajectory_validator::plugin::ValidatorInterface"),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  const auto filters = params_.filter_names;
  for (const auto & filter : filters) {
    load_metric(filter);
  }

  constexpr bool shadow_mode = true;
  for (const auto & filter : params_.shadow_mode_filter_names) {
    load_metric(filter, shadow_mode);
  }

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectoryValidator::map_callback, this, std::placeholders::_1));

  sub_trajectories_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryValidator::process, this, std::placeholders::_1));

  pub_trajectories_ = create_publisher<CandidateTrajectories>("~/output/trajectories", 1);

  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/feasible_trajectory_filter", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  pub_validation_reports_ = std::make_shared<autoware_utils_debug::DebugPublisher>(this, "~/debug");
}

void TrajectoryValidator::process(const CandidateTrajectories::ConstSharedPtr msg)
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

  context.traffic_light_signals = sub_traffic_lights_.take_data();

  context.lanelet_map = lanelet_map_ptr_;
  if (!context.lanelet_map) {
    return;
  }

  if (listener_.is_old(params_)) {
    params_ = listener_.get_params();

    for (const auto & plugin : plugins_) {
      plugin->update_parameters(params_);
    }
    RCLCPP_INFO(get_logger(), "Dynamic parameters updated successfully.");
  }

  diagnostics_interface_.clear();
  evaluation_tables_.clear();
  size_t num_feasible_trajectories = 0;

  // Create output message for filtered trajectories
  const auto uuid_to_name = get_generator_uuid_to_name_map(*msg);

  auto filtered_msg = std::make_unique<CandidateTrajectories>();

  std::vector<ValidationReport> reports;

  for (const auto & trajectory : msg->candidate_trajectories) {
    // Apply each filter to the trajectory

    EvaluationTable table;
    const auto hex_generator_id = autoware_utils_uuid::to_hex_string(trajectory.generator_id);
    table.generator_id = hex_generator_id;

    std::vector<MetricReport> metrics;
    for (const auto & plugin : plugins_) {
      PluginEvaluation evaluation;
      evaluation.plugin_name = plugin->get_name();
      evaluation.is_shadow_mode = plugin->is_shadow_mode();

      const auto res = plugin->is_feasible(trajectory.points, context);
      if (!res) {
        evaluation.is_feasible = false;
        evaluation.reason = res.error();
      } else {
        const auto & val = res.value();
        evaluation.is_feasible = evaluation.is_feasible && val.is_feasible;
        if (!val.is_feasible) {
          evaluation.reason = "Found failed metrics";
        }
        metrics.insert(metrics.end(), val.metrics.begin(), val.metrics.end());
      }

      if (!evaluation.is_feasible) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "[%s] %s", plugin->get_name().c_str(),
          evaluation.reason.c_str());
      }

      diagnostics_interface_.add_key_value(
        plugin->get_name(), evaluation.is_feasible ? std::string("OK") : std::string("NG"));
      table.evaluations[plugin->category()].push_back(evaluation);
    }

    evaluation_tables_.push_back(table);

    if (table.all_acceptable()) filtered_msg->candidate_trajectories.push_back(trajectory);

    const auto all_feasible = table.all_feasible();
    if (all_feasible) ++num_feasible_trajectories;

    reports.push_back(
      autoware_trajectory_validator::build<ValidationReport>()
        .trajectory_stamp(trajectory.header.stamp)
        .generator_id(trajectory.generator_id)
        .generator_name(uuid_to_name.at(hex_generator_id))
        .level(all_feasible ? ValidationReport::OK : ValidationReport::ERROR)
        .metrics(std::move(metrics)));
  }

  // Also filter generator_info to match kept trajectories
  for (const auto & traj : filtered_msg->candidate_trajectories) {
    auto it = std::find_if(
      msg->generator_info.begin(), msg->generator_info.end(),
      [&](const auto & info) { return traj.generator_id.uuid == info.generator_id.uuid; });

    if (it != msg->generator_info.end()) {
      filtered_msg->generator_info.push_back(*it);
    }
  }

  update_diagnostic(*msg, num_feasible_trajectories);
  publish_validation_reports(reports);
  pub_trajectories_->publish(*filtered_msg);
}

void TrajectoryValidator::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
}

void TrajectoryValidator::load_metric(const std::string & name, const bool is_shadow_mode)
{
  if (name.empty()) return;

  try {
    auto plugin = plugin_loader_.createSharedInstance(name);

    for (const auto & p : plugins_) {
      if (plugin->get_name() == p->get_name()) {
        RCLCPP_WARN_STREAM(get_logger(), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    plugin->set_vehicle_info(vehicle_info_);
    plugin->set_shadow_mode(is_shadow_mode);
    plugin->update_parameters(params_);
    std::string category;
    size_t pos = name.find("::");
    if (pos != std::string::npos) {
      category = name.substr(0, pos);
    }
    plugin->set_category(category);

    plugins_.push_back(plugin);

    RCLCPP_INFO_STREAM(
      get_logger(), "The scene plugin '" << name << "' is loaded and initialized.");
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "[validator] createSharedInstance failed for '" << name << "': " << e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "[validator] unexpected exception for '" << name << "': " << e.what());
  }
}

void TrajectoryValidator::unload_metric(const std::string & name)
{
  auto it = std::remove_if(
    plugins_.begin(), plugins_.end(),
    [&](const std::shared_ptr<plugin::ValidatorInterface> & plugin) {
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

void TrajectoryValidator::update_diagnostic(
  const CandidateTrajectories & input_trajectories, const size_t num_feasible_trajectories)
{
  if (input_trajectories.candidate_trajectories.size() == num_feasible_trajectories) {
    // All trajectories are feasible
    diagnostics_interface_.update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
  } else if (num_feasible_trajectories == 0) {
    // No feasible trajectories found
    diagnostics_interface_.update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No feasible trajectories found");
  } else {
    // At least one trajectory is infeasible
    diagnostics_interface_.update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "At least one trajectory is infeasible");
  }

  diagnostics_interface_.publish(this->get_clock()->now());
}

void TrajectoryValidator::publish_validation_reports(const std::vector<ValidationReport> & reports)
{
  auto msg = autoware_trajectory_validator::build<ValidationReportArray>().reports(reports);
  pub_validation_reports_->publish<ValidationReportArray>("validation_reports", msg);
}
}  // namespace autoware::trajectory_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_validator::TrajectoryValidator)
