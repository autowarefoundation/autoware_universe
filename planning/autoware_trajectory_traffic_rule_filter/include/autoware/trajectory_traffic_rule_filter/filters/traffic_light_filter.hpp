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

#ifndef AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__FILTERS__TRAFFIC_LIGHT_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__FILTERS__TRAFFIC_LIGHT_FILTER_HPP_

#include "autoware/trajectory_traffic_rule_filter/traffic_rule_filter_interface.hpp"

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/Forward.h>

#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter::plugin
{

class TrafficLightFilter : public TrafficRuleFilterInterface
{
public:
  TrafficLightFilter();

  tl::expected<void, std::string> is_feasible(const TrajectoryPoints & trajectory_points) override;
  void set_traffic_lights(
    const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr & traffic_lights)
    override;

  void set_parameters(const traffic_rule_filter::Params & params) override { params_ = params; }

  /// @brief return true if ego can safely pass an amber traffic light
  [[nodiscard]] bool can_pass_amber_light(
    const double distance_to_stop_line, const double current_velocity,
    const double current_acceleration, const double time_to_cross_stop_line) const;

private:
  /// @brief return the red and amber stop lines related to the given lanelets
  [[nodiscard]] std::pair<
    std::vector<lanelet::BasicLineString2d>, std::vector<lanelet::BasicLineString2d>>
  get_stop_lines(const lanelet::Lanelets & lanelets) const;

  autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr traffic_lights_;
  traffic_rule_filter::Params params_;
};

}  // namespace autoware::trajectory_traffic_rule_filter::plugin

#endif  // AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__FILTERS__TRAFFIC_LIGHT_FILTER_HPP_
