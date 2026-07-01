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

// NOLINTNEXTLINE
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_TEMPORAL_MPT_OPTIMIZER_UTILS_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_TEMPORAL_MPT_OPTIMIZER_UTILS_HPP_

#include "autoware/trajectory_optimizer/acados_interface.hpp"

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <array>
#include <cstddef>
#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_temporal_mpt_optimizer_utils
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

struct TemporalMPTReferences
{
  size_t start_idx{0};
  size_t terminal_idx{0};
  double x_offset{0.0};
  double y_offset{0.0};
  double psi_bias{0.0};
  std::array<double, temporal_mpt::NX> x0_local{};
  std::array<double, temporal_mpt::NY> yref_stage0{};
  std::array<std::array<double, temporal_mpt::NY>, temporal_mpt::N - 1> stage_yrefs{};
  std::array<double, temporal_mpt::NYN> terminal_yref{};
};

/**
 * @brief Returns the trajectory index whose XY position is closest to (x, y).
 */
size_t find_closest_trajectory_index(const TrajectoryPoints & traj_points, double x, double y);

/**
 * @brief Returns a 2π multiple aligning reference yaw at start_idx with initial heading x0_yaw.
 */
double compute_yaw_psi_bias(double x0_yaw, double yaw_at_start);

/**
 * @brief Builds ego-centered LINEAR_LS references for the temporal MPC horizon.
 *
 * Horizon sampling uses the incoming trajectory as a time-ordered discrete sequence:
 * start_idx is the index closest to x0 position, then start_idx + k for stage k.
 */
TemporalMPTReferences build_temporal_mpt_references(
  const TrajectoryPoints & traj_points, const std::array<double, temporal_mpt::NX> & x0);

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_temporal_mpt_optimizer_utils

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_TEMPORAL_MPT_OPTIMIZER_UTILS_HPP_
