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

#ifndef RUN_OUT_MODULE_HPP_
#define RUN_OUT_MODULE_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <autoware/universe_utils/system/time_keeper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{
class RunOutModule : public PluginModuleInterface
{
public:
  RunOutModule() = default;
  /// @brief initialize the module
  void init(rclcpp::Node & node, const std::string & module_name) override;
  /// @brief initialize the parameters
  void init_parameters(rclcpp::Node & node);
  /// @brief update the parameters
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  /// @brief plan change of velocity based on detected collisions with objects running out on the
  /// ego trajectory
  VelocityPlanningResult plan(
    [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;
  std::string get_module_name() const override { return module_name_; }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.predicted_objects = true;
    return required_subscription_info;
  }
  void publish_planning_factor() override { planning_factor_interface_->publish(); };

private:
  inline static const std::string ns_ = "run_out";
  std::string module_name_{"uninitialized"};
  rclcpp::Clock::SharedPtr clock_{nullptr};
  // TODO(Maxime): move to the module interface
  rclcpp::Publisher<universe_utils::ProcessingTimeDetail>::SharedPtr timekeeper_publisher_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr debug_trajectory_publisher_;
  std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;
  std::optional<diagnostic_updater::Updater> diagnostic_updater_ = std::nullopt;
  std::unique_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>
    objects_of_interest_marker_interface_;
  std::optional<double> unfeasible_stop_deceleration_;

  run_out::ObjectDecisionsTracker decisions_tracker_;
  run_out::Parameters params_{};

  /// @brief update whether we are currently inserting a stop that breaks the deceleration limit
  void update_unfeasible_stop_status(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /// @brief publish a debug trajectory with the calculated slowdowns added to trajectory input used
  /// by the module
  void publish_debug_trajectory(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
    const VelocityPlanningResult & planning_result);
  /// @brief populate the planning factors based on the module's planning result
  void add_planning_factors(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
    const run_out::RunOutResult & result,
    const std::unordered_map<std::string, autoware_internal_planning_msgs::msg::SafetyFactor> &
      safety_factor_per_object);
};
}  // namespace autoware::motion_velocity_planner

#endif  // RUN_OUT_MODULE_HPP_
