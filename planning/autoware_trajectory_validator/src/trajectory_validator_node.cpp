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

  std::sort(plugins_.begin(), plugins_.end(), [](const auto & plugin1, const auto & plugin2) {
    return plugin1->get_name() < plugin2->get_name();
  });

  subscribers();
  publishers();
}

void TrajectoryValidator::subscribers()
{
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectoryValidator::map_callback, this, std::placeholders::_1));

  sub_trajectories_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryValidator::process, this, std::placeholders::_1));
}

void TrajectoryValidator::publishers()
{
  pub_trajectories_ = create_publisher<CandidateTrajectories>("~/output/trajectories", 1);

  pub_processing_time_detail_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/trajectory_validator_node", 1);
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(pub_processing_time_detail_);

  pub_validation_reports_ = std::make_shared<autoware_utils_debug::DebugPublisher>(this, "~/debug");
  pub_debug_ = std::make_shared<autoware_utils_debug::DebugPublisher>(this, "~/debug");
}

tl::expected<FilterContext, std::string> TrajectoryValidator::take_data()
{
  FilterContext context;

  context.odometry = sub_odometry_.take_data();
  if (!context.odometry) {
    return tl::make_unexpected("Failed to take odometry data");
  }

  context.predicted_objects = sub_objects_.take_data();
  if (!context.predicted_objects) {
    return tl::make_unexpected("Failed to take predicted objects data");
  }

  context.acceleration = sub_acceleration_.take_data();
  if (!context.acceleration) {
    return tl::make_unexpected("Failed to take acceleration data");
  }

  context.traffic_light_signals = sub_traffic_lights_.take_data();

  context.lanelet_map = lanelet_map_ptr_;
  if (!context.lanelet_map) {
    return tl::make_unexpected("Lanelet map is not available");
  }

  if (context.lanelet_map->laneletLayer.empty()) {
    return tl::make_unexpected("Lanelet map does not contain any lanelets");
  }

  return context;
}

void TrajectoryValidator::process(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  std::unordered_map<std::string, double> processing_time_ms;

  stop_watch.tic("Total");

  auto context_opt = take_data();
  if (!context_opt) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", context_opt.error().c_str());
    return;
  }

  const FilterContext & context = context_opt.value();

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

  CandidateTrajectories filtered_msg;

  std::vector<ValidationReport> reports;

  for (const auto & trajectory : msg->candidate_trajectories) {
    EvaluationTable table;
    const auto hex_generator_id = autoware_utils_uuid::to_hex_string(trajectory.generator_id);
    table.generator_id = hex_generator_id;

    std::vector<MetricReport> metrics;
    for (const auto & plugin : plugins_) {
      PluginEvaluation evaluation;
      evaluation.plugin_name = plugin->get_name();
      evaluation.is_shadow_mode = plugin->is_shadow_mode();
      stop_watch.tic(evaluation.plugin_name);

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
      processing_time_ms[evaluation.plugin_name] += stop_watch.toc(evaluation.plugin_name);

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

    if (table.all_acceptable()) filtered_msg.candidate_trajectories.push_back(trajectory);

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
  for (const auto & traj : filtered_msg.candidate_trajectories) {
    auto it = std::find_if(
      msg->generator_info.begin(), msg->generator_info.end(),
      [&](const auto & info) { return traj.generator_id.uuid == info.generator_id.uuid; });

    if (it != msg->generator_info.end()) {
      filtered_msg.generator_info.push_back(*it);
    }
  }

  pub_trajectories_->publish(filtered_msg);

  update_diagnostic(*msg, num_feasible_trajectories);

  publish_validation_reports(reports);

  processing_time_ms["Total"] = stop_watch.toc("Total");

  publish_debug(processing_time_ms, context.odometry->pose.pose);
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

    if (!plugin) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to create plugin instance for '" << name << "'.");
      return;
    }

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

void TrajectoryValidator::publish_debug(
  const std::unordered_map<std::string, double> & processing_time,
  const geometry_msgs::msg::Pose & marker_pose)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  publish_plugins_debug_markers();
  publish_plugins_report_text(marker_pose);
  publish_processing_time(processing_time);
  publish_processing_time_text(processing_time);
}

void TrajectoryValidator::publish_plugins_debug_markers() const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & plugin : plugins_) {
    auto plugin_markers = plugin->take_debug_markers();
    pub_debug_->publish<visualization_msgs::msg::MarkerArray>(
      "markers/" + plugin->get_name(), plugin_markers);
  }
}

void TrajectoryValidator::publish_plugins_report_text(const geometry_msgs::msg::Pose & marker_pose)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  std::unordered_map<std::string, int> used_filters;
  for (const auto & eval : evaluation_tables_) {
    for (const auto & [category, evaluations] : eval.evaluations) {
      for (const auto & plugin_eval : evaluations) {
        if (!plugin_eval.is_feasible) {
          used_filters[plugin_eval.plugin_name]++;
        }
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
    "map", get_clock()->now(), "plugin_report", 0,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    autoware_utils_visualization::create_marker_scale(0.0, 0.0, 0.4),
    autoware_utils_visualization::create_marker_color(1., 1., 1., 0.999));

  plugin_report_text.pose = marker_pose;
  plugin_report_text.pose.position.z += vehicle_info_.vehicle_height_m;
  plugin_report_text.text = fmt::to_string(out);

  visualization_msgs::msg::MarkerArray plugin_report_text_marker;
  plugin_report_text_marker.markers.push_back(plugin_report_text);
  pub_debug_->publish<visualization_msgs::msg::MarkerArray>(
    "plugin_report_text", plugin_report_text_marker);
}

void TrajectoryValidator::publish_processing_time(
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

void TrajectoryValidator::publish_processing_time_text(
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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_validator::TrajectoryValidator)
