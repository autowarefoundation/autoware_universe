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

#ifndef AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__OUT_OF_LANE_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__OUT_OF_LANE_FILTER_HPP_

#include "autoware/trajectory_safety_filter/safety_filter_interface.hpp"

#include <autoware/boundary_departure_checker/boundary_departure_checker.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>

namespace autoware::trajectory_safety_filter::plugin
{

// Parameters for OutOfLaneFilter
struct OutOfLaneParams
{
  double max_check_time = 3.0;  // seconds - max time horizon to check
  double min_value = 0.0;       // meters - minimum distance to lane boundary
};

class OutOfLaneFilter : public SafetyFilterInterface
{
public:
  OutOfLaneFilter();

  tl::expected<void, std::string> is_feasible(
    const TrajectoryPoints & traj_points, const FilterContext & context) final;

  void set_parameters(rclcpp::Node & node) final;

  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) final;

private:
  OutOfLaneParams params_;
  std::unique_ptr<autoware::boundary_departure_checker::BoundaryDepartureChecker>
    boundary_departure_checker_;
};
}  // namespace autoware::trajectory_safety_filter::plugin

#endif  // AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__OUT_OF_LANE_FILTER_HPP_
