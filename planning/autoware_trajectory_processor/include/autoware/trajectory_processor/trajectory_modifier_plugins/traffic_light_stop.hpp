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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__TRAFFIC_LIGHT_STOP_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__TRAFFIC_LIGHT_STOP_HPP_

#include "autoware/traffic_light_compliance_checker/structs.hpp"
#include "autoware/traffic_light_compliance_checker/traffic_light_compliance_checker.hpp"
#include "autoware/trajectory_processor/plugin_base.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_utils/utils.hpp"

#include <autoware_trajectory_processor/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

namespace autoware::trajectory_modifier::plugin
{
using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using visualization_msgs::msg::MarkerArray;

class TrafficLightStop : public PluginBase
{
public:
  TrafficLightStop() = default;

  bool modify_trajectory(TrajectoryPoints & traj_points, const InputData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const InputData & input);

  void update_params(const TrajectoryModifierParams & params) override;

  const TrajectoryModifierParams::TrafficLightStop & get_params() const { return params_; }

protected:
  void set_up_params() override;

private:
  TrajectoryModifierParams::TrafficLightStop params_;
  TrajectoryModifierParams::StoppingConstraints stopping_params_;

  std::optional<autoware::traffic_light_compliance_checker::Violation> nearest_violation_;

  std::unique_ptr<autoware::traffic_light_compliance_checker::TrafficLightComplianceChecker>
    checker_;

  struct DebugData
  {
    bool active = false;
    size_t violations_count = 0;
    double nearest_violation_arc_length = 0.0;
    double stop_point_arc_length = 0.0;
  } debug_data_;

  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_text_;

  bool check_traffic_lights(const TrajectoryPoints & traj_points, const InputData & input);

  bool set_stop_point(TrajectoryPoints & traj_points, const InputData & input);

  bool check_inputs(const InputData & input);

  void publish_debug_string() const;
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__TRAFFIC_LIGHT_STOP_HPP_
