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

#ifndef AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__TRAFFIC_RULE_FILTER_INTERFACE_HPP_
#define AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__TRAFFIC_RULE_FILTER_INTERFACE_HPP_

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter::plugin
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware::vehicle_info_utils::VehicleInfo;

class TrafficRuleFilterInterface
{
public:
  explicit TrafficRuleFilterInterface(std::string name) : name_(std::move(name)) {}
  virtual ~TrafficRuleFilterInterface() = default;
  TrafficRuleFilterInterface(const TrafficRuleFilterInterface &) = delete;
  TrafficRuleFilterInterface & operator=(const TrafficRuleFilterInterface &) = delete;
  TrafficRuleFilterInterface(TrafficRuleFilterInterface &&) = delete;
  TrafficRuleFilterInterface & operator=(TrafficRuleFilterInterface &&) = delete;

  virtual bool is_feasible(const TrajectoryPoints & trajectory_points) = 0;
  std::string get_name() const { return name_; }

protected:
  std::string name_;
  std::shared_ptr<VehicleInfo> vehicle_info_ptr_;
};
}  // namespace autoware::trajectory_traffic_rule_filter::plugin

#endif  // AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__TRAFFIC_RULE_FILTER_INTERFACE_HPP_
