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

#include "autoware/trajectory_validator/validator_interface.hpp"

#include <lanelet2_core/Forward.h>

#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::traffic_rule
{
class TrafficLightFilter : public ValidatorInterface
{
public:
  TrafficLightFilter();

  result_t is_feasible(const TrajectoryPoints & traj_points, const FilterContext & context) final;

  void update_parameters(const validator::Params & params) final;

  /// @brief return true if ego can safely pass an amber traffic light
  /// @note made public for testing purposes
  [[nodiscard]] bool can_pass_amber_light(
    const double distance_to_stop_line, const double current_velocity,
    const double current_acceleration, const double time_to_cross_stop_line) const;

private:
  /// @brief return the red and amber stop lines related to the given traffic light groups
  [[nodiscard]] std::pair<
    std::vector<lanelet::BasicLineString2d>, std::vector<lanelet::BasicLineString2d>>
  get_stop_lines(
    const lanelet::LaneletMap & lanelet_map,
    const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_lights) const;
  /// @brief return true if there is a stop point and it is within margin distance of the stop line
  [[nodiscard]] bool is_stop_point_within_margin_from_stop_line(
    const std::optional<TrajectoryPoint> & stop_point,
    const lanelet::BasicLineString2d & stop_line) const;

  validator::Params::TrafficLight params_;
};

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__TRAFFIC_LIGHT_FILTER_HPP_
