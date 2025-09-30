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

#ifndef AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__FILTERS__STOP_LINE_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__FILTERS__STOP_LINE_FILTER_HPP_

#include "autoware/trajectory_traffic_rule_filter/traffic_rule_filter_interface.hpp"

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter::plugin
{

class StopLineFilter : public TrafficRuleFilterInterface
{
public:
  StopLineFilter();

  bool is_feasible(const TrajectoryPoints & trajectory_points) override;

private:
  lanelet::ConstLanelets get_lanelets_from_trajectory(
    const TrajectoryPoints & trajectory_points) const;
};

}  // namespace autoware::trajectory_traffic_rule_filter::plugin

#endif  // AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__FILTERS__STOP_LINE_FILTER_HPP_
