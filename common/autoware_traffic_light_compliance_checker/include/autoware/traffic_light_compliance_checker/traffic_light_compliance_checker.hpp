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

#ifndef AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__TRAFFIC_LIGHT_COMPLIANCE_CHECKER_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__TRAFFIC_LIGHT_COMPLIANCE_CHECKER_HPP_

#include "autoware/traffic_light_compliance_checker/structs.hpp"
#include "autoware/traffic_light_compliance_checker/traffic_light_status_tracker.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::traffic_light_compliance_checker
{

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

  ~TrafficLightComplianceChecker();

  /**
   * @brief check if the trajectory complies with traffic lights
   * @param input input data for compliance check (raw signals are filtered internally)
   * @return result of compliance check, or error message if check fails
   */
  [[nodiscard]] tl::expected<ComplianceResult, std::string> check(const Inputs & input);

  /**
   * @brief update parameters
   * @param parameters new parameters
   */
  void update_parameters(const Parameters & parameters);

private:
  [[nodiscard]] std::vector<int64_t> get_force_reject_amber_ids(
    const rclcpp::Time & current_time, bool is_ego_stopped) const;

  void update_amber_rejection_history(
    const ComplianceResult & result, const rclcpp::Time & current_time,
    const std::vector<int64_t> & force_reject_amber_ids);

  void cleanup_amber_rejection_history(const rclcpp::Time & current_time);

  [[nodiscard]] tl::expected<ComplianceResult, std::string> check_with_filtered_signals(
    const Inputs & input,
    const autoware_perception_msgs::msg::TrafficLightGroupArray & filtered_signals,
    const std::vector<int64_t> & force_reject_amber_ids) const;

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
  std::unique_ptr<TrafficLightStatusTracker> status_tracker_;
  std::unordered_map<int64_t, rclcpp::Time> amber_rejection_history_;
};

}  // namespace autoware::traffic_light_compliance_checker

#endif  // AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__TRAFFIC_LIGHT_COMPLIANCE_CHECKER_HPP_
