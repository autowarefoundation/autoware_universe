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

#ifndef PLANNING__AUTOWARE_TRAJECTORY_PROCESSOR__TESTS__PARAMETER_UPDATE_TEST_ACCESSOR_HPP_
#define PLANNING__AUTOWARE_TRAJECTORY_PROCESSOR__TESTS__PARAMETER_UPDATE_TEST_ACCESSOR_HPP_

#include "autoware/trajectory_processor/trajectory_modifier.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_plugins/obstacle_stop.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_plugins/stop_point_fixer.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_plugins/surround_obstacle_stop.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_plugins/traffic_light_stop.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_plugins/velocity_modifier.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_eb_smoother_optimizer.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_extender.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_kinematic_feasibility_enforcer.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_mpt_optimizer.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_point_fixer.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_qp_smoother.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_spline_smoother.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_temporal_mpt_optimizer.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_velocity_optimizer.hpp"

#include <memory>
#include <vector>

namespace autoware::trajectory_optimizer
{

class ParameterUpdateTestAccessor
{
public:
  using Base = plugin::TrajectoryOptimizerPluginBase;

  static const TrajectoryOptimizerParams & params(const TrajectoryOptimizer & node)
  {
    return node.params_;
  }

  static const std::vector<std::shared_ptr<Base>> & plugins(const TrajectoryOptimizer & node)
  {
    return node.plugins_;
  }

  static bool enabled(const Base & plugin) { return plugin.enabled_; }

  static const auto & params(const plugin::TrajectoryPointFixer & plugin)
  {
    return plugin.fixer_params_;
  }

  static const auto & params(const plugin::TrajectoryKinematicFeasibilityEnforcer & plugin)
  {
    return plugin.feasibility_params_;
  }

  static const auto & params(const plugin::TrajectoryQPSmoother & plugin)
  {
    return plugin.qp_params_;
  }

  static const auto & params(const plugin::TrajectorySplineSmoother & plugin)
  {
    return plugin.spline_params_;
  }

  static const auto & params(const plugin::TrajectoryExtender & plugin)
  {
    return plugin.extender_params_;
  }

  static const auto & params(const plugin::TrajectoryVelocityOptimizer & plugin)
  {
    return plugin.velocity_params_;
  }

  static const auto & params(const plugin::TrajectoryMPTOptimizer & plugin)
  {
    return plugin.mpt_params_;
  }

  static const auto & trajectory_params(const plugin::TrajectoryMPTOptimizer & plugin)
  {
    return plugin.traj_param_;
  }

  static const auto & nearest_params(const plugin::TrajectoryMPTOptimizer & plugin)
  {
    return plugin.ego_nearest_param_;
  }

  static const auto & params(const plugin::TrajectoryTemporalMPTOptimizer & plugin)
  {
    return plugin.mpt_params_;
  }

  static const auto & common_params(const plugin::TrajectoryEBSmootherOptimizer & plugin)
  {
    return plugin.common_param_;
  }

  static const auto & nearest_params(const plugin::TrajectoryEBSmootherOptimizer & plugin)
  {
    return plugin.ego_nearest_param_;
  }
};

}  // namespace autoware::trajectory_optimizer

namespace autoware::trajectory_modifier
{

class ParameterUpdateTestAccessor
{
public:
  using Base = plugin::TrajectoryModifierPluginBase;

  static const trajectory_modifier_params::Params & params(const TrajectoryModifier & node)
  {
    return node.params_;
  }

  static const std::vector<std::shared_ptr<Base>> & plugins(const TrajectoryModifier & node)
  {
    return node.plugins_;
  }

  static bool enabled(const Base & plugin) { return plugin.enabled_; }
  static double trajectory_time_step(const Base & plugin) { return plugin.trajectory_time_step_; }

  static const auto & params(const plugin::StopPointFixer & plugin) { return plugin.params_; }

  static const auto & params(const plugin::VelocityModifier & plugin) { return plugin.params_; }

  static const auto & params(const plugin::ObstacleStop & plugin) { return plugin.params_; }
  static const auto & stopping_params(const plugin::ObstacleStop & plugin)
  {
    return plugin.stopping_params_;
  }

  static const auto & params(const plugin::SurroundObstacleStop & plugin) { return plugin.params_; }

  static const auto & params(const plugin::TrafficLightStop & plugin) { return plugin.params_; }
  static const auto & stopping_params(const plugin::TrafficLightStop & plugin)
  {
    return plugin.stopping_params_;
  }
};

}  // namespace autoware::trajectory_modifier

#endif  // PLANNING__AUTOWARE_TRAJECTORY_PROCESSOR__TESTS__PARAMETER_UPDATE_TEST_ACCESSOR_HPP_
