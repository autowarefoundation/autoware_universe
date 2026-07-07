// Copyright 2026 TIER IV, Inc.
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

#ifndef PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__SURROUND_OBSTACLE_STOP_HPP_
#define PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__SURROUND_OBSTACLE_STOP_HPP_

#include "autoware/obstacle_proximity_checker/obstacle_proximity_checker.hpp"
#include "plugin_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{
using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class SurroundObstacleStop : public PluginInterface
{
public:
  SurroundObstacleStop() = default;

  void run(TrajectoryPoints & traj_points, const ModifierData & data) override;

  void update_params(const MinimumRuleBasedPlannerParams & params) override;

  const MinimumRuleBasedPlannerParams::SurroundObstacleStop & get_params() const { return params_; }

protected:
  void on_initialize(const MinimumRuleBasedPlannerParams & params) override;

private:
  MinimumRuleBasedPlannerParams::SurroundObstacleStop params_;

  std::unique_ptr<obstacle_proximity_checker::ProximityChecker> proximity_checker_;

  std::optional<obstacle_proximity_checker::CheckResult> proximity_check_result_;

  bool is_stop_active_{false};
  std::optional<rclcpp::Time> last_obstacle_found_time_;

  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_text_;

  [[nodiscard]] bool check_inputs(const ModifierData & data) const;

  [[nodiscard]] obstacle_proximity_checker::Inputs to_proximity_checker_inputs(
    const ModifierData & data) const;

  [[nodiscard]] bool is_obstacle_nearby(const ModifierData & data);

  [[nodiscard]] bool is_stop_required(
    const TrajectoryPoints & traj_points, const ModifierData & data);

  std::optional<geometry_msgs::msg::TransformStamped> get_transform(
    const std::string & target, const std::string & source, const rclcpp::Time & stamp,
    double duration_sec) const;

  void set_stop_point(TrajectoryPoints & traj_points, const ModifierData & data);

  void publish_debug_string(bool is_active) const;
};

}  // namespace autoware::minimum_rule_based_planner::plugin

#endif  // PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__SURROUND_OBSTACLE_STOP_HPP_
