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

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_eb_smoother_optimizer.hpp"

#include "autoware/trajectory_processor/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

void TrajectoryEBSmootherOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, TrajectoryOptimizerData & data)
{
  if (!enabled_) {
    return;
  }
  utils::smooth_trajectory_with_elastic_band(
    traj_points, data.current_odometry, eb_path_smoother_ptr_);

  autoware::motion_utils::calculate_time_from_start(
    traj_points, data.current_odometry.pose.pose.position);
}

void TrajectoryEBSmootherOptimizer::on_initialize(const TrajectoryOptimizerParams & params)
{
  auto node_ptr = get_node_ptr();
  enabled_ = params.use_eb_smoother;
  ego_nearest_param_ = EgoNearestParam(node_ptr);
  common_param_ = CommonParam(node_ptr);
  smoother_time_keeper_ptr_ = std::make_shared<SmootherTimekeeper>();
  eb_path_smoother_ptr_ = std::make_shared<EBPathSmoother>(
    node_ptr, false, ego_nearest_param_, common_param_, smoother_time_keeper_ptr_);
  eb_path_smoother_ptr_->initialize(false, common_param_);
  eb_path_smoother_ptr_->resetPreviousData();
}

void TrajectoryEBSmootherOptimizer::update_params(const TrajectoryOptimizerParams & params)
{
  enabled_ = params.use_eb_smoother;
  // TODO(Maxime): support parameter updates of internal objects
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectoryEBSmootherOptimizer,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
