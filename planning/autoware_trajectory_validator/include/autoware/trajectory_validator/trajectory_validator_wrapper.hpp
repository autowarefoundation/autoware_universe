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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_WRAPPER_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_WRAPPER_HPP_

#include "autoware/trajectory_validator/detail/trajectory_validator.hpp"
#include "autoware/trajectory_validator/detail/trajectory_validator_report.hpp"
#include "autoware/trajectory_validator/detail/validator_context.hpp"
#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware_trajectory_validator/msg/metric_report.hpp>
#include <autoware_trajectory_validator/msg/validation_report.hpp>
#include <autoware_trajectory_validator/msg/validation_report_array.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_trajectory_validator::msg::MetricReport;
using autoware_trajectory_validator::msg::ValidationReport;
using autoware_trajectory_validator::msg::ValidationReportArray;
using autoware_utils_diagnostics::DiagnosticsInterface;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

/**
 * @brief Adapter for TrajectoryValidator: manages plugin loading, parameter updates,
 * diagnostics, and debug publishing.
 */
class TrajectoryValidatorWrapper
{
public:
  /**
   * @brief Loads plugins declared in the parameter lists and initialises all publishers.
   * @param node Node used for publisher creation and logging.
   * @param node_parameters_interface Parameter interface for declaring and reading parameters.
   * @param vehicle_info Ego vehicle dimensions forwarded to each plugin.
   * @param time_keeper Shared time keeper for processing time tracking.
   */
  TrajectoryValidatorWrapper(
    rclcpp::Node & node,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
    vehicle_info_utils::VehicleInfo vehicle_info,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper);

  /**
   * @brief Runs all plugins against the input trajectories and returns the feasible subset.
   * @param input_trajectories Candidate trajectories to validate.
   * @param context Current world state snapshot.
   */
  CandidateTrajectories validate_trajectories(
    const CandidateTrajectories & input_trajectories, const ValidatorContext & context);

private:
  /** @brief Creates the debug publisher. */
  void publishers();

  /** @brief Reloads parameters from the parameter server if they have changed. */
  void update_parameters();

  /**
   * @brief Loads a single plugin by class name and configures it.
   * @param name Plugin class name to load.
   * @param is_shadow_mode If true, plugin results are logged but not enforced.
   */
  void load_metric(const std::string & name, const bool is_shadow_mode = false);

  /**
   * @brief Updates the diagnostics status based on the number of feasible trajectories.
   * @param input_trajectories Original input trajectories (used for total count).
   * @param num_feasible_trajectories Number of trajectories that passed all plugins.
   */
  void update_diagnostic(
    const CandidateTrajectories & input_trajectories, const size_t num_feasible_trajectories);

  /**
   * @brief Publishes the validation report array.
   * @param reports Per-trajectory validation reports to publish.
   */
  void publish_validation_reports(const std::vector<ValidationReport> & reports);

  /**
   * @brief Publishes all debug outputs: markers, report text, and processing times.
   * @param evaluation_tables Per-trajectory plugin evaluation results.
   * @param processing_time Per-plugin elapsed time in milliseconds.
   * @param marker_pose Pose used to position text markers in RViz.
   */
  void publish_debug(
    const std::vector<EvaluationTable> & evaluation_tables,
    const std::unordered_map<std::string, double> & processing_time,
    const geometry_msgs::msg::Pose & marker_pose);

  /** @brief Publishes each plugin's accumulated debug markers. */
  void publish_plugins_debug_markers() const;

  /**
   * @brief Publishes a text marker summarising how many trajectories each plugin filtered.
   * @param evaluation_tables Per-trajectory plugin evaluation results.
   * @param marker_pose Pose used to position the text marker in RViz.
   */
  void publish_plugins_report_text(
    const std::vector<EvaluationTable> & evaluation_tables,
    const geometry_msgs::msg::Pose & marker_pose);

  /**
   * @brief Publishes each plugin's processing time as a scalar stamped value.
   * @param processing_time Per-plugin elapsed time in milliseconds.
   */
  void publish_processing_time(const std::unordered_map<std::string, double> & processing_time);

  /**
   * @brief Publishes all plugin processing times as a single text marker.
   * @param processing_time Per-plugin elapsed time in milliseconds.
   */
  void publish_processing_time_text(
    const std::unordered_map<std::string, double> & processing_time);

  rclcpp::Node * node_ptr_{nullptr};
  std::string interface_name_{"trajectory_validator"};
  rclcpp::Logger logger_;
  validator::ParamListener validator_params_listener_;
  validator::Params validator_params_;
  vehicle_info_utils::VehicleInfo vehicle_info_;
  std::unique_ptr<TrajectoryValidator> validator_ptr_;

  // Plugin infrastructure
  pluginlib::ClassLoader<plugin::ValidatorInterface> plugin_loader_;
  std::vector<std::shared_ptr<plugin::ValidatorInterface>> plugins_;

  // Publishers
  std::shared_ptr<autoware_utils_debug::DebugPublisher> pub_debug_;

  // Internal state
  std::unique_ptr<DiagnosticsInterface> diagnostics_interface_ptr_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_WRAPPER_HPP_
