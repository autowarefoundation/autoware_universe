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

#include "autoware/trajectory_safety_filter/safety_filter_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <vector>

namespace autoware::trajectory_safety_filter::plugin
{
class DummyFilter : public SafetyFilterInterface
{
  struct DummyFilterParam
  {
    double dummy_param{0.0};
  };

public:
  DummyFilter() : SafetyFilterInterface("DummyFilter") {}

  tl::expected<void, std::string> is_feasible(
    const TrajectoryPoints & traj_points, const FilterContext & /*context*/) final
  {
    if (traj_points.empty()) {
      return tl::make_unexpected("Empty trajectory");
    }

    // Magic trigger: If we set velocity to -999.0 in our test, simulate a plugin rejection
    if (traj_points.front().longitudinal_velocity_mps == -999.0) {
      return tl::make_unexpected("Dummy filter explicitly rejected this trajectory");
    }

    return {};  // All other trajectories are feasible
  }

  void set_parameters(rclcpp::Node & node) final
  {
    using autoware_utils_rclcpp::get_or_declare_parameter;
    params_.dummy_param = get_or_declare_parameter<double>(node, "dummy.dummy_param");
  }

  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) final
  {
    using autoware_utils_rclcpp::update_param;

    update_param<double>(parameters, "dummy.dummy_param", params_.dummy_param);
  }

private:
  DummyFilterParam params_;
};
}  // namespace autoware::trajectory_safety_filter::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_safety_filter::plugin::DummyFilter,
  autoware::trajectory_safety_filter::plugin::SafetyFilterInterface)
