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
#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_VELOCITY_OPTIMIZER_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_VELOCITY_OPTIMIZER_HPP_

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/plugin_utils/continuous_jerk_smoother.hpp"
#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_processor::plugin
{
using autoware::trajectory_processor::TrajectoryProcessorData;
using autoware::trajectory_processor::TrajectoryProcessorParams;
using autoware::trajectory_processor::plugin::ProcessingResult;
using autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware::trajectory_processor::plugin::ContinuousJerkSmoother;
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

class TrajectoryVelocityOptimizer : public TrajectoryProcessorPluginBase
{
public:
  TrajectoryVelocityOptimizer() = default;
  ~TrajectoryVelocityOptimizer() = default;

  ProcessingResult process(TrajectoryPoints & traj_points, TrajectoryProcessorData & data) override;
  void update_params(const TrajectoryProcessorParams & params) override;

protected:
  void on_initialize(const TrajectoryProcessorParams & params) override;

private:
  std::shared_ptr<ContinuousJerkSmoother> continuous_jerk_smoother_{nullptr};
  TrajectoryVelocityOptimizerParams velocity_params_;
  autoware::agnocast_wrapper::polling::PollingSubscriber<VelocityLimit>::SharedPtr
    sub_planning_velocity_;
  PublisherHandle<VelocityLimit> pub_velocity_limit_;
};
}  // namespace autoware::trajectory_processor::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_VELOCITY_OPTIMIZER_HPP_
