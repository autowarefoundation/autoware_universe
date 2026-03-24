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

struct Params
{
  double deceleration_limit;  // trajectories crossing an amber light are rejected if ego can stop
                              // at the stop line without breaking this limit
  double jerk_limit;  // trajectories crossing an amber light are rejected if ego can stop at the
                      // stop line without breaking this limit
  double
    delay_response_time;  // delay response time used to estimate the minimum ego stopping distance
  double crossing_time_limit;  // trajectories crossing an amber light are rejected if they cannot
                               // cross before this time
  bool treat_amber_light_as_red_light;  // when true, amber lights are handled like red lights
  struct CheckedTrajectoryLength
  {
    double deceleration_limit;  // deceleration limit to calculate the stop distance used as
                                // trajectory length limit
    double jerk_limit;          // jerk limit to calculate the stop distance used as trajectory
                                // length limit
  } checked_trajectory_length;
};

class TrafficLightFilter : public ValidatorInterface
{
public:
  TrafficLightFilter();

  tl::expected<void, std::string> is_feasible(
    const TrajectoryPoints & traj_points, const FilterContext & context) final;

  void set_parameters(rclcpp::Node & node) final;

  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) final;

  /// @brief return true if ego can safely pass an amber traffic light
  /// @note made public for testing purposes
  [[nodiscard]] bool can_pass_amber_light(
    const double distance_to_stop_line, const double current_velocity,
    const double current_acceleration, const double time_to_cross_stop_line) const;

private:
  /// @brief return the red and amber stop lines related to the given lanelets
  [[nodiscard]] std::pair<
    std::vector<lanelet::BasicLineString2d>, std::vector<lanelet::BasicLineString2d>>
  get_stop_lines(
    const lanelet::Lanelets & lanelets,
    const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_lights) const;

  Params params_{};
};

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__TRAFFIC_LIGHT_FILTER_HPP_
