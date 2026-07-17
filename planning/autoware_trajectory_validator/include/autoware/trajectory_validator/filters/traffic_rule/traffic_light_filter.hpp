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

#include <autoware/traffic_light_compliance_checker/structs.hpp>
#include <autoware/traffic_light_compliance_checker/traffic_light_compliance_checker.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator::plugin::traffic_rule
{
class TrafficLightFilter : public ValidatorInterface
{
public:
  TrafficLightFilter();

  result_t is_feasible(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) final;

  void update_parameters(const validator::Params & params) final;

  void set_vehicle_info(const VehicleInfo & vehicle_info) final;

private:
  struct RejectionInfo
  {
    geometry_msgs::msg::Point stop_line_pos;
    std::string signal_label;
    size_t rejection_count{0};
  };

  std::unique_ptr<traffic_light_compliance_checker::TrafficLightComplianceChecker> checker_;
  validator::Params::TrafficLight params_;

  std::unordered_map<int64_t, RejectionInfo> aggregated_rejections_;
  std::optional<rclcpp::Time> last_frame_time_;

  void update_debug_data(
    const std::vector<traffic_light_compliance_checker::StopLineCrossing> & violations,
    const std::vector<traffic_light_compliance_checker::CrossingCommitmentDebugInfo> &
      crossing_commitments,
    const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_light_signals,
    const rclcpp::Time & current_time, const double z);
};

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__TRAFFIC_LIGHT_FILTER_HPP_
