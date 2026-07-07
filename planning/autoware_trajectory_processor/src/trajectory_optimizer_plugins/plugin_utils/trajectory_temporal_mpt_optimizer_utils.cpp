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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_temporal_mpt_optimizer_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

namespace autoware::trajectory_optimizer::plugin::trajectory_temporal_mpt_optimizer_utils
{

size_t find_closest_trajectory_index(const TrajectoryPoints & traj_points, double x, double y)
{
  geometry_msgs::msg::Point query;
  query.x = x;
  query.y = y;
  query.z = 0.0;

  const auto closest_itr = std::min_element(
    traj_points.begin(), traj_points.end(), [&query](const auto & a, const auto & b) {
      return autoware_utils_geometry::calc_distance2d(a.pose.position, query) <
             autoware_utils_geometry::calc_distance2d(b.pose.position, query);
    });

  return static_cast<size_t>(std::distance(traj_points.begin(), closest_itr));
}

double compute_yaw_psi_bias(double x0_yaw, double yaw_at_start)
{
  constexpr double two_pi = 2.0 * M_PI;
  return std::round((x0_yaw - yaw_at_start) / two_pi) * two_pi;
}

TemporalMPTReferences build_temporal_mpt_references(
  const TrajectoryPoints & traj_points, const std::array<double, temporal_mpt::NX> & x0)
{
  TemporalMPTReferences refs;

  const size_t n_pts = traj_points.size();
  refs.start_idx = find_closest_trajectory_index(traj_points, x0[0], x0[1]);

  const double yaw_at_start = tf2::getYaw(traj_points.at(refs.start_idx).pose.orientation);
  refs.psi_bias = compute_yaw_psi_bias(x0[2], yaw_at_start);

  refs.x_offset = x0[0];
  refs.y_offset = x0[1];

  refs.yref_stage0 = {x0[0] - refs.x_offset, x0[1] - refs.y_offset, x0[2], x0[3], 0.0, 0.0};

  for (size_t k = 1; k < temporal_mpt::N; ++k) {
    const size_t idx = std::min(refs.start_idx + k, n_pts - 1);
    const auto & p = traj_points.at(idx);
    const double yaw = tf2::getYaw(p.pose.orientation) + refs.psi_bias;
    const double v_ref = std::max(0.0, static_cast<double>(p.longitudinal_velocity_mps));
    refs.stage_yrefs[k - 1] = {
      p.pose.position.x - refs.x_offset, p.pose.position.y - refs.y_offset, yaw, v_ref, 0.0, 0.0};
  }

  refs.terminal_idx = std::min(refs.start_idx + temporal_mpt::N, n_pts - 1);
  const auto & terminal_point = traj_points.at(refs.terminal_idx);
  const double terminal_yaw = tf2::getYaw(terminal_point.pose.orientation) + refs.psi_bias;
  const double terminal_v_ref =
    std::max(0.0, static_cast<double>(terminal_point.longitudinal_velocity_mps));
  refs.terminal_yref = {
    terminal_point.pose.position.x - refs.x_offset, terminal_point.pose.position.y - refs.y_offset,
    terminal_yaw, terminal_v_ref};

  refs.x0_local = {x0[0] - refs.x_offset, x0[1] - refs.y_offset, x0[2], x0[3]};

  return refs;
}

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_temporal_mpt_optimizer_utils
