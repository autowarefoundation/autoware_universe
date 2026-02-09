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

// NOLINTNEXTLINE
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_VELOCITY_OPTIMIZER_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_VELOCITY_OPTIMIZER_HPP_

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/continuous_jerk_smoother.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware::trajectory_optimizer::plugin::ContinuousJerkSmoother;
using autoware_internal_planning_msgs::msg::VelocityLimit;

struct TrajectoryVelocityOptimizerParams
{
  double nearest_dist_threshold_m{1.5};
  double nearest_yaw_threshold_deg{60.0};
  double target_pull_out_speed_mps{1.0};
  double target_pull_out_acc_mps2{1.0};
  double max_lateral_accel_mps2{1.5};
  double min_limited_speed_mps{3.0};      // Minimum speed when applying lateral acceleration limit
  double default_max_velocity_mps{8.33};  // 30 km/h
  bool set_engage_speed{false};
  bool limit_speed{true};
  bool limit_lateral_acceleration{false};
  bool smooth_velocities{false};

  // Continuous jerk smoother parameters
  ContinuousJerkSmootherParams continuous_jerk_smoother_params;
};

class TrajectoryVelocityOptimizer : public TrajectoryOptimizerPluginBase
{
public:
  TrajectoryVelocityOptimizer() = default;
  ~TrajectoryVelocityOptimizer() = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper) override;
  void optimize_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
    const TrajectoryOptimizerData & data) override;
  void set_up_params() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  std::shared_ptr<ContinuousJerkSmoother> continuous_jerk_smoother_{nullptr};
  TrajectoryVelocityOptimizerParams velocity_params_;
  std::shared_ptr<autoware_utils_rclcpp::InterProcessPollingSubscriber<VelocityLimit>>
    sub_planning_velocity_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;
};
}  // namespace autoware::trajectory_optimizer::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_VELOCITY_OPTIMIZER_HPP_
