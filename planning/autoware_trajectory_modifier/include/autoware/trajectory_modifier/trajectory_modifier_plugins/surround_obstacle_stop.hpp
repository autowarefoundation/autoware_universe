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
#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp"

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace autoware::trajectory_modifier::plugin
{
using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::TrajectoryModifierParams;
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using ModifierParams = trajectory_modifier_params::Params;
using autoware_internal_debug_msgs::msg::StringStamped;
using utils::obstacle_stop::PointCloud;

class SurroundObstacleStop : public TrajectoryModifierPluginBase
{
public:
  SurroundObstacleStop() = default;

  ProcessingResult process(TrajectoryPoints & traj_points, TrajectoryModifierData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const TrajectoryModifierData & input);

  void update_params(const TrajectoryModifierParams & params) override;

  const ModifierParams::SurroundObstacleStop & get_params() const { return params_; }

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  ModifierParams::SurroundObstacleStop params_;

  std::unique_ptr<obstacle_proximity_checker::ProximityChecker> proximity_checker_;
  std::unique_ptr<utils::obstacle_stop::PointCloudFilter> pointcloud_filter_;
  std::optional<obstacle_proximity_checker::CheckResult> proximity_check_result_;

  std::optional<rclcpp::Time> last_frame_time_;

  bool is_stop_active_{false};
  std::optional<rclcpp::Time> last_obstacle_found_time_;

  PublisherHandle<StringStamped> pub_debug_text_;

  [[nodiscard]] bool check_inputs(const TrajectoryModifierData & input) const;

  [[nodiscard]] obstacle_proximity_checker::Inputs to_proximity_checker_inputs(
    const TrajectoryModifierData & input) const;

  [[nodiscard]] bool is_obstacle_nearby(const TrajectoryModifierData & input);

  std::optional<geometry_msgs::msg::TransformStamped> get_transform(
    const std::string & source, const std::string & target, const rclcpp::Time & stamp,
    double duration_sec) const;

  void publish_debug_string(bool is_active) const;
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__SURROUND_OBSTACLE_STOP_HPP_
