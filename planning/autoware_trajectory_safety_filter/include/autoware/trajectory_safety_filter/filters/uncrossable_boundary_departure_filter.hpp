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

#ifndef AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__UNCROSSABLE_BOUNDARY_DEPARTURE_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__UNCROSSABLE_BOUNDARY_DEPARTURE_FILTER_HPP_

#include "autoware/trajectory_safety_filter/safety_filter_interface.hpp"

#include <autoware/boundary_departure_checker/uncrossable_boundary_departure_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <any>
#include <memory>
#include <string>
#include <unordered_map>
namespace autoware::trajectory_safety_filter::plugin
{
class UncrossableBoundaryDepartureFilter : public SafetyFilterInterface
{
public:
  UncrossableBoundaryDepartureFilter() : SafetyFilterInterface("UncrossableBoundaryDepartureFilter")
  {
  }

  bool is_feasible(const TrajectoryPoints & traj_points, const FilterContext & context) final;
  void set_parameters(
    [[maybe_unused]] const std::unordered_map<std::string, std::any> & params) final {};

private:
  std::unique_ptr<autoware::boundary_departure_checker::UncrossableBoundaryDepartureChecker>
    uncrossable_boundary_departure_checker_ptr_;
  rclcpp::Logger log_ = rclcpp::get_logger(name_);
  std::shared_ptr<rclcpp::Clock> clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  [[nodiscard]] std::optional<std::string> is_invalid_input(
    const TrajectoryPoints & traj_points, const FilterContext & context) const;

  template <typename... Args>
  void warn_throttle(const char * fmt)
  {
    RCLCPP_WARN_THROTTLE(log_, *clock_, 5000, fmt);
  }

  template <typename... Args>
  void warn_throttle(const char * fmt, Args... args)
  {
    RCLCPP_WARN_THROTTLE(log_, *clock_, 5000, fmt, args...);
  }
};
}  // namespace autoware::trajectory_safety_filter::plugin

#endif  // AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__UNCROSSABLE_BOUNDARY_DEPARTURE_FILTER_HPP_
