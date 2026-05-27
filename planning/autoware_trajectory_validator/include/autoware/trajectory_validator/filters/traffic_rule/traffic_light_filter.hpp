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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__TRAFFIC_LIGHT_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__TRAFFIC_LIGHT_FILTER_HPP_

#include "autoware/trajectory_validator/filters/traffic_rule/traffic_light_compliance_checker.hpp"
#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator::plugin::traffic_rule
{
class TrafficLightFilter : public ValidatorInterface
{
public:
  TrafficLightFilter();

  result_t is_feasible(const TrajectoryPoints & traj_points, const FilterContext & context) final;

  void update_parameters(const validator::Params & params) final;

  void set_vehicle_info(const VehicleInfo & vehicle_info) final;

private:
  struct SignalStateHistory
  {
    autoware_perception_msgs::msg::TrafficLightGroup msg;
    rclcpp::Time first_seen_time;
    rclcpp::Time last_seen_time;
  };

  std::unique_ptr<traffic_light_filter::TrafficLightComplianceChecker> checker_;
  validator::Params::TrafficLight params_;

  std::unordered_map<int64_t, SignalStateHistory> signal_history_;
  std::unordered_map<int64_t, rclcpp::Time> amber_rejection_history_;

  autoware_perception_msgs::msg::TrafficLightGroupArray filter_signals(
    const autoware_perception_msgs::msg::TrafficLightGroupArray & signals,
    const rclcpp::Time & current_time, bool is_ego_stopped);

  std::vector<int64_t> get_force_reject_amber_ids(
    const rclcpp::Time & current_time, bool is_ego_stopped) const;

  void cleanup_history(const rclcpp::Time & current_time);
};

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__TRAFFIC_LIGHT_FILTER_HPP_
