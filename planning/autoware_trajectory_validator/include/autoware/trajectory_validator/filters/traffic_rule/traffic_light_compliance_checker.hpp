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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::traffic_light_filter
{

/// @brief input data for traffic light compliance check
struct Inputs
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory;
  lanelet::LaneletMapPtr map;
  autoware_planning_msgs::msg::LaneletRoute route;
  autoware_perception_msgs::msg::TrafficLightGroupArray signals;
  double current_velocity;
  double current_acceleration;
  std::vector<int64_t> force_reject_amber_ids;
};

/// @brief information about a stop line and its associated traffic light
struct StopLineInfo
{
  lanelet::BasicLineString2d line;
  int64_t traffic_light_id;
};

/// @brief type of traffic light violation
enum class ViolationType { RED_LIGHT, AMBER_LIGHT };

/// @brief violation detail
struct Violation
{
  ViolationType type;
  lanelet::BasicLineString2d stop_line;
  int64_t traffic_light_id;
};

/// @brief result of compliance check
struct ComplianceResult
{
  std::vector<Violation> violations;
};

/// @brief parameters for traffic light compliance check
struct Parameters
{
  double deceleration_limit;
  double jerk_limit;
  double delay_response_time;
  double crossing_time_limit;
  bool treat_amber_light_as_red_light;
  double stop_overshoot_margin;
  double stable_duration_threshold_red;
  double stable_duration_threshold_amber;
  double amber_rejection_hysteresis_duration;
  double ego_stopped_velocity_threshold;
  struct CheckedTrajectoryLength
  {
    double deceleration_limit;
    double jerk_limit;
  } checked_trajectory_length;
};

/// @brief class to check if a trajectory complies with traffic lights
class TrafficLightComplianceChecker
{
public:
  /**
   * @brief constructor
   * @param parameters parameters for compliance check
   * @param vehicle_info vehicle information
   */
  TrafficLightComplianceChecker(
    const Parameters & parameters, const vehicle_info_utils::VehicleInfo & vehicle_info);

  /**
   * @brief check if the trajectory complies with traffic lights
   * @param input input data for compliance check
   * @return result of compliance check, or error message if check fails
   */
  [[nodiscard]] tl::expected<ComplianceResult, std::string> check(const Inputs & input) const;

  /**
   * @brief update parameters
   * @param parameters new parameters
   */
  void update_parameters(const Parameters & parameters);

private:
  /// @brief return the red and amber stop lines related to the given traffic light groups
  [[nodiscard]] std::pair<std::vector<StopLineInfo>, std::vector<StopLineInfo>> get_stop_lines(
    const lanelet::LaneletMap & lanelet_map,
    const autoware_planning_msgs::msg::LaneletRoute & route,
    const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_lights) const;

  /// @brief return true if there is a stop point and it is within margin distance of the stop line
  [[nodiscard]] bool is_stop_point_within_margin_from_stop_line(
    const std::optional<lanelet::BasicPoint2d> & stop_point,
    const lanelet::BasicLineString2d & stop_line) const;

  /// @brief return true if ego can safely pass an amber traffic light
  [[nodiscard]] bool can_pass_amber_light(
    const double distance_to_stop_line, const double current_velocity,
    const double current_acceleration, const double time_to_cross_stop_line) const;

  Parameters params_;
  vehicle_info_utils::VehicleInfo vehicle_info_;
};

}  // namespace autoware::trajectory_validator::traffic_light_filter
// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER_HPP_
