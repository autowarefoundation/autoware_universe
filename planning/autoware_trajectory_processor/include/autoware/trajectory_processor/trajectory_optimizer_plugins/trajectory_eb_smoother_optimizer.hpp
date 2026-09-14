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
#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_EB_SMOOTHER_OPTIMIZER_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_EB_SMOOTHER_OPTIMIZER_HPP_
#include "autoware/path_smoother/elastic_band.hpp"
#include "autoware/path_smoother/replan_checker.hpp"
#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <memory>
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
using autoware::path_smoother::CommonParam;
using autoware::path_smoother::EBPathSmoother;
using autoware::path_smoother::EgoNearestParam;
using autoware::path_smoother::ReplanChecker;
using SmootherTimekeeper = autoware::path_smoother::TimeKeeper;

class TrajectoryEBSmootherOptimizer : public TrajectoryProcessorPluginBase
{
public:
  TrajectoryEBSmootherOptimizer() = default;
  ~TrajectoryEBSmootherOptimizer() = default;

  ProcessingResult process(TrajectoryPoints & traj_points, TrajectoryProcessorData & data) override;
  void update_params(const TrajectoryProcessorParams & params) override;

protected:
  void on_initialize(const TrajectoryProcessorParams & params) override;

private:
  CommonParam common_param_{};
  EgoNearestParam ego_nearest_param_;
  std::shared_ptr<EBPathSmoother> eb_path_smoother_ptr_{nullptr};
  mutable std::shared_ptr<SmootherTimekeeper> smoother_time_keeper_ptr_{nullptr};
};
}  // namespace autoware::trajectory_processor::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_EB_SMOOTHER_OPTIMIZER_HPP_
