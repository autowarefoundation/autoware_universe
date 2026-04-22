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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_NODE_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_NODE_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_trajectory_validator/autoware_trajectory_validator_param.hpp>
#include <autoware_trajectory_validator/msg/metric_report.hpp>
#include <autoware_trajectory_validator/msg/validation_report.hpp>
#include <autoware_trajectory_validator/msg/validation_report_array.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <functional>
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

class TrajectoryValidator : public rclcpp::Node
{
public:
  struct PluginEvaluation
  {
    std::string plugin_name;
    bool is_feasible{true};
    bool is_shadow_mode{false};
    std::string reason;
  };

  struct EvaluationTable
  {
    std::string generator_id;
    std::unordered_map<std::string, std::vector<PluginEvaluation>> evaluations;

    /**
     * @brief Returns true if any evaluation is feasible or in shadow mode.
     */
    bool all_acceptable() const
    {
      return all_evaluations([](const auto & e) { return e.is_feasible || e.is_shadow_mode; });
    }

    /**
     * @brief Returns true if all evaluations are feasible.
     */
    bool all_feasible() const
    {
      return all_evaluations([](const auto & e) { return e.is_feasible; });
    }

  private:
    /**
     * @brief Returns true if all evaluations satisfy the given predicate.
     */
    bool all_evaluations(const std::function<bool(const PluginEvaluation &)> & pred) const
    {
      return std::all_of(evaluations.begin(), evaluations.end(), [&](const auto & pair) {
        return std::all_of(pair.second.begin(), pair.second.end(), pred);
      });
    }
  };

  explicit TrajectoryValidator(const rclcpp::NodeOptions & node_options);

private:
  /**
   * @brief Initialise the node's subscribers.
   */
  void subscribers();

  /**
   * @brief Initialise the node's publishers.
   */
  void publishers();

  /**
   * @brief Gather the latest inputs required to run the filter plugins.
   */
  tl::expected<FilterContext, std::string> take_data();

  void process(const CandidateTrajectories::ConstSharedPtr msg);

  void map_callback(const LaneletMapBin::ConstSharedPtr msg);

  void load_metric(const std::string & name, const bool is_shadow_mode = false);

  /**
   * @brief Unloads a metric plugin
   * @param name Metric plugin name to unload
   */
  void unload_metric(const std::string & name);
  void update_diagnostic(
    const CandidateTrajectories & input_trajectories, const size_t num_feasible_trajectories);

  /**
   * @brief Publishes validation reports
   * @param reports Validation reports to publish
   */
  void publish_validation_reports(const std::vector<ValidationReport> & reports);

  /**
   * @brief Publish the union of all debug information.
   */
  void publish_debug(
    const std::unordered_map<std::string, double> & processing_time,
    const geometry_msgs::msg::Pose & marker_pose);

  /**
   * @brief Publish each plugin's debug markers.
   */
  void publish_plugins_debug_markers() const;

  /**
   * @brief Publish each plugin's filtering report in a single string stamped marker.
   */
  void publish_plugins_report_text(const geometry_msgs::msg::Pose & marker_pose);

  /**
   * @brief Publish each plugin's processing time as scalar value.
   * @param processing_time Map of plugin name -> elapsed time in [ms].
   */
  void publish_processing_time(const std::unordered_map<std::string, double> & processing_time);

  /**
   * @brief Publish each plugin's processing time in a single string stamped marker.
   * @param processing_time Map of plugin name -> elapsed time in [ms].
   */
  void publish_processing_time_text(
    const std::unordered_map<std::string, double> & processing_time);

  // Parameters
  validator::ParamListener listener_;
  validator::Params params_;

  // Plugin infrastructure
  pluginlib::ClassLoader<plugin::ValidatorInterface> plugin_loader_;
  std::vector<std::shared_ptr<plugin::ValidatorInterface>> plugins_;

  // Subscribers
  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<PredictedObjects> sub_objects_{
    this, "~/input/objects"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    sub_acceleration_{this, "~/input/acceleration"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    sub_traffic_lights_{this, "~/input/traffic_signals"};
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<CandidateTrajectories>::SharedPtr sub_trajectories_;

  // Publishers
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;
  std::shared_ptr<autoware_utils_debug::DebugPublisher> pub_validation_reports_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    pub_processing_time_detail_;
  std::shared_ptr<autoware_utils_debug::DebugPublisher> pub_debug_;

  // Internal state
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  std::vector<EvaluationTable> evaluation_tables_;
  DiagnosticsInterface diagnostics_interface_{this, "trajectory_validator"};
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_NODE_HPP_
