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

#include "autoware/trajectory_validator/trajectory_validator_wrapper.hpp"

#include "autoware/trajectory_validator/detail/trajectory_validator.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

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

namespace autoware::trajectory_validator
{

TrajectoryValidatorWrapper::TrajectoryValidatorWrapper(
  rclcpp::Node & node,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
  vehicle_info_utils::VehicleInfo vehicle_info,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: node_ptr_(&node),
  logger_(node.get_logger().get_child(interface_name_)),
  validator_params_listener_{node_parameters_interface},
  vehicle_info_(vehicle_info),
  plugin_loader_(
    "autoware_trajectory_validator", "autoware::trajectory_validator::plugin::ValidatorInterface"),
  time_keeper_(std::move(time_keeper))
{
  if (!time_keeper_) {
    throw std::runtime_error("TimeKeeper is required for TrajectoryValidatorWrapper");
  }

  validator_params_ = validator_params_listener_.get_params();
  const auto filters = validator_params_.filter_names;
  for (const auto & filter : filters) {
    load_metric(filter);
  }

  constexpr bool shadow_mode = true;
  for (const auto & filter : validator_params_.shadow_mode_filter_names) {
    load_metric(filter, shadow_mode);
  }

  std::sort(plugins_.begin(), plugins_.end(), [](const auto & plugin1, const auto & plugin2) {
    return plugin1->get_name() < plugin2->get_name();
  });
  publishers();

  validator_ptr_ = std::make_unique<TrajectoryValidator>(plugins_);
  diagnostics_interface_ptr_ = std::make_unique<DiagnosticsInterface>(node_ptr_, interface_name_);
}

void TrajectoryValidatorWrapper::load_metric(const std::string & name, const bool is_shadow_mode)
{
  if (name.empty()) return;

  try {
    auto plugin = plugin_loader_.createSharedInstance(name);

    if (!plugin) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to create plugin instance for '" << name << "'.");
      return;
    }

    for (const auto & p : plugins_) {
      if (plugin->get_name() == p->get_name()) {
        RCLCPP_WARN_STREAM(logger_, "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    plugin->set_vehicle_info(vehicle_info_);
    plugin->set_shadow_mode(is_shadow_mode);
    plugin->update_parameters(validator_params_);

    std::string category;
    size_t pos = name.find("::");
    if (pos != std::string::npos) {
      category = name.substr(0, pos);
    }
    plugin->set_category(category);

    plugins_.push_back(plugin);

    RCLCPP_INFO_STREAM(logger_, "The validator plugin '" << name << "' is loaded and initialized.");
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(logger_, "createSharedInstance failed for '" << name << "': " << e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, "unexpected exception for '" << name << "': " << e.what());
  }
}

void TrajectoryValidatorWrapper::update_parameters()
{
  if (validator_params_listener_.is_old(validator_params_)) {
    validator_params_ = validator_params_listener_.get_params();
    validator_ptr_->update_parameters(validator_params_);

    RCLCPP_INFO(logger_, "Trajectory Validator parameters are updated.");
  }
}

void TrajectoryValidatorWrapper::publishers()
{
  pub_debug_ = std::make_shared<autoware_utils_debug::DebugPublisher>(node_ptr_, "~/debug");
}

CandidateTrajectories TrajectoryValidatorWrapper::validate_trajectories(
  const autoware_internal_planning_msgs::msg::CandidateTrajectories & input_trajectories,
  const ValidatorContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  update_parameters();

  const auto report = validator_ptr_->process(input_trajectories, context);

  diagnostics_interface_ptr_->clear();

  for (const auto & table : report.evaluation_tables) {
    for (const auto & eval : table.plugin_evaluations) {
      if (!eval.is_feasible) {
        RCLCPP_WARN_THROTTLE(
          logger_, *node_ptr_->get_clock(), 1000, "[%s] %s", eval.plugin_name.c_str(),
          eval.reason.c_str());
      }
      // Exact original behavior: last trajectory's result overwrites previous ones
      diagnostics_interface_ptr_->add_key_value(
        eval.plugin_name, std::string(eval.is_feasible ? "OK" : "NG"));
    }
  }

  update_diagnostic(input_trajectories, report.num_feasible_trajectories);

  publish_validation_reports(report.validation_reports);

  // Wire up the debug publishers using the opaque report data
  publish_debug(report.evaluation_tables, report.processing_time_ms, context.odometry->pose.pose);

  return report.valid_trajectories;
}

void TrajectoryValidatorWrapper::update_diagnostic(
  const CandidateTrajectories & input_trajectories, const size_t num_feasible_trajectories)
{
  if (input_trajectories.candidate_trajectories.size() == num_feasible_trajectories) {
    // All trajectories are feasible
    diagnostics_interface_ptr_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::OK, "");
  } else if (num_feasible_trajectories == 0) {
    // No feasible trajectories found
    diagnostics_interface_ptr_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No feasible trajectories found");
  } else {
    // At least one trajectory is infeasible
    diagnostics_interface_ptr_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "At least one trajectory is infeasible");
  }

  diagnostics_interface_ptr_->publish(node_ptr_->get_clock()->now());
}

void TrajectoryValidatorWrapper::publish_validation_reports(
  const std::vector<ValidationReport> & reports)
{
  auto msg = autoware_trajectory_validator::build<ValidationReportArray>().reports(reports);
  pub_debug_->publish<ValidationReportArray>("validation_reports", msg);
}

void TrajectoryValidatorWrapper::publish_debug(
  const std::vector<EvaluationTable> & evaluation_tables,
  const std::unordered_map<std::string, double> & processing_time,
  const geometry_msgs::msg::Pose & marker_pose)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  publish_plugins_debug_markers();
  publish_plugins_report_text(evaluation_tables, marker_pose);
  publish_processing_time(processing_time);
  publish_processing_time_text(processing_time);
}

void TrajectoryValidatorWrapper::publish_plugins_debug_markers() const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & plugin : plugins_) {
    auto plugin_markers = plugin->take_debug_markers();
    pub_debug_->publish<visualization_msgs::msg::MarkerArray>(
      "markers/" + plugin->get_name(), plugin_markers);
  }
}

void TrajectoryValidatorWrapper::publish_plugins_report_text(
  const std::vector<EvaluationTable> & evaluation_tables,
  const geometry_msgs::msg::Pose & marker_pose)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  std::unordered_map<std::string, int> used_filters;
  for (const auto & eval : evaluation_tables) {
    for (const auto & plugin_eval : eval.plugin_evaluations) {
      if (!plugin_eval.is_feasible) {
        used_filters[plugin_eval.plugin_name]++;
      }
    }
  }

  fmt::memory_buffer out;
  auto out_it = std::back_inserter(out);

  fmt::format_to(out_it, "{:^50}\n", "--- Trajectory Validator Filtering Result ---");

  int skip_counter = 0;
  for (const auto & plugin : plugins_) {
    if (!plugin) continue;

    const auto & name = plugin->get_name();
    const auto it = used_filters.find(name);

    if (it != used_filters.end()) {
      fmt::format_to(out_it, "- {}: {} path(s) filtered\n", name, it->second);
    } else {
      ++skip_counter;
    }
  }

  for (int i = 0; i < skip_counter; ++i) {
    fmt::format_to(out_it, "\n");
  }

  fmt::format_to(out_it, "-----------------------------------");

  auto plugin_report_text = autoware_utils_visualization::create_default_marker(
    "map", node_ptr_->get_clock()->now(), "plugin_report", 0,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    autoware_utils_visualization::create_marker_scale(0.0, 0.0, 0.4),
    autoware_utils_visualization::create_marker_color(1., 1., 1., 0.999));

  plugin_report_text.pose = marker_pose;
  plugin_report_text.pose.position.z += vehicle_info_.vehicle_height_m;
  plugin_report_text.text = fmt::to_string(out);
  plugin_report_text.frame_locked = true;

  visualization_msgs::msg::MarkerArray plugin_report_text_marker;
  plugin_report_text_marker.markers.push_back(plugin_report_text);
  pub_debug_->publish<visualization_msgs::msg::MarkerArray>(
    "plugin_report_text", plugin_report_text_marker);
}

void TrajectoryValidatorWrapper::publish_processing_time(
  const std::unordered_map<std::string, double> & processing_time)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & [key, value] : processing_time) {
    if (key == "Total") {
      pub_debug_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "processing_time_ms", value);
      continue;
    }
    pub_debug_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      key + "/processing_time_ms", value);
  }
}

void TrajectoryValidatorWrapper::publish_processing_time_text(
  const std::unordered_map<std::string, double> & processing_time)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  fmt::memory_buffer out;
  auto time_report = std::back_inserter(out);
  fmt::format_to(time_report, "\n--- Trajectory Validator Processing Time Report ---\n");

  const auto get_time = [&](const auto & key) {
    auto it = processing_time.find(key);
    return (it != processing_time.end()) ? it->second : 0.0;
  };

  fmt::format_to(time_report, "Total: {:.2f} ms\n", get_time("Total"));

  for (const auto & plugin : plugins_) {
    if (!plugin) continue;

    const auto & plugin_name = plugin->get_name();

    fmt::format_to(time_report, "- {}: {:.2f} ms\n", plugin_name, get_time(plugin_name));
  }

  pub_debug_->publish<autoware_internal_debug_msgs::msg::StringStamped>(
    "processing_time_text", fmt::to_string(out));
}
}  // namespace autoware::trajectory_validator
