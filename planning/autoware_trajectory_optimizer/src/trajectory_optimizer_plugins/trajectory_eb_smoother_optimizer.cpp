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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_eb_smoother_optimizer.hpp"

#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

TrajectoryEBSmootherOptimizer::TrajectoryEBSmootherOptimizer(
  const std::string name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  const TrajectoryOptimizerParams & params)
: TrajectoryOptimizerPluginBase(name, node_ptr, time_keeper, params)
{
  // parameters for ego nearest search
  ego_nearest_param_ = EgoNearestParam(node_ptr);
  // parameters for trajectory
  common_param_ = CommonParam(node_ptr);
  smoother_time_keeper_ptr_ = std::make_shared<SmootherTimekeeper>();
  eb_path_smoother_ptr_ = std::make_shared<EBPathSmoother>(
    node_ptr, false, ego_nearest_param_, common_param_, smoother_time_keeper_ptr_);
  eb_path_smoother_ptr_->initialize(false, common_param_);
  eb_path_smoother_ptr_->resetPreviousData();
}

void TrajectoryEBSmootherOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  const TrajectoryOptimizerData & data)
{
  // Use elastic band to smooth the trajectory
  if (params.use_eb_smoother) {
    utils::smooth_trajectory_with_elastic_band(
      traj_points, data.current_odometry, eb_path_smoother_ptr_);
  }
}

void TrajectoryEBSmootherOptimizer::set_up_params()
{
}

rcl_interfaces::msg::SetParametersResult TrajectoryEBSmootherOptimizer::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  {  // parameters for ego nearest search
    ego_nearest_param_.onParam(parameters);

    // parameters for trajectory
    common_param_.onParam(parameters);

    // parameters for core algorithms
    eb_path_smoother_ptr_->onParam(parameters);
    eb_path_smoother_ptr_->initialize(false, common_param_);
    eb_path_smoother_ptr_->resetPreviousData();
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin
