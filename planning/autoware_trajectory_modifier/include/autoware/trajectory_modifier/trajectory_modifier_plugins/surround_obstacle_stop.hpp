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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__SURROUND_OBSTACLE_STOP_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__SURROUND_OBSTACLE_STOP_HPP_

#include "autoware/obstacle_proximity_checker/obstacle_proximity_checker.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>

#include <memory>
#include <optional>

namespace autoware::trajectory_modifier::plugin
{
using autoware_internal_debug_msgs::msg::StringStamped;

class SurroundObstacleStop : public TrajectoryModifierPluginBase
{
public:
  SurroundObstacleStop() = default;

  bool modify_trajectory(TrajectoryPoints & traj_points, const InputData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const InputData & input) override;

  void update_params(const TrajectoryModifierParams & params) override;

  const TrajectoryModifierParams::SurroundObstacleStop & get_params() const { return params_; }

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  TrajectoryModifierParams::SurroundObstacleStop params_;

  std::unique_ptr<obstacle_proximity_checker::ProximityChecker> proximity_checker_;

  bool is_stop_active_{false};
  std::optional<rclcpp::Time> last_obstacle_found_time_;

  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_text_;

  [[nodiscard]] bool check_inputs(const InputData & input) const;

  [[nodiscard]] obstacle_proximity_checker::Inputs to_proximity_checker_inputs(
    const InputData & input) const;

  [[nodiscard]] bool is_obstacle_nearby(const InputData & input);

  void publish_debug_string(bool is_active) const;
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__SURROUND_OBSTACLE_STOP_HPP_
