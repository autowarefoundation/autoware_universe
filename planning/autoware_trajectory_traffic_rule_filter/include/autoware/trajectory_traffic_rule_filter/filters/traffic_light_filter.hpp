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

#ifndef AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__FILTERS__TRAFFIC_LIGHT_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__FILTERS__TRAFFIC_LIGHT_FILTER_HPP_

#include "autoware/trajectory_traffic_rule_filter/traffic_rule_filter_interface.hpp"

#include <autoware_utils_rclcpp/polling_subscriber.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <any>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter::plugin
{
struct TrafficSignalStamped
{
  builtin_interfaces::msg::Time stamp;
  autoware_perception_msgs::msg::TrafficLightGroup signal;
};

class TrafficLightFilter : public TrafficRuleFilterInterface
{
public:
  TrafficLightFilter();

  bool is_feasible(const TrajectoryPoints & trajectory_points) override;

  void set_traffic_lights();

  void set_lanelet_map(const lanelet::LaneletMapPtr & lanelet_map);

private:
  autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr traffic_lights_;
  lanelet::LaneletMapPtr lanelet_map_;

  bool check_traffic_light_collision(const TrajectoryPoints & trajectory_points) const;

  std::map<int64_t, TrafficSignalStamped> traffic_light_id_map_;

  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    traffic_signals_subscriber_{this, "~/input/traffic_signals"};
};

}  // namespace autoware::trajectory_traffic_rule_filter::plugin

#endif  // AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__FILTERS__TRAFFIC_LIGHT_FILTER_HPP_
