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

#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_MPT_OPTIMIZER_HPP_
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_MPT_OPTIMIZER_HPP_

#include "autoware/path_optimizer/common_structs.hpp"
#include "autoware/path_optimizer/mpt_optimizer.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_mpt_optimizer_utils.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

using autoware::path_optimizer::DebugData;
using autoware::path_optimizer::EgoNearestParam;
using autoware::path_optimizer::MPTOptimizer;
using autoware::path_optimizer::PlannerData;
using autoware::path_optimizer::TrajectoryParam;

struct MPTParams
{
  // Bounds generation
  double corridor_width_m{3.5};
  bool enable_adaptive_width{true};
  double curvature_width_factor{0.5};
  double velocity_width_factor{0.3};
  double min_clearance_m{0.5};

  // MPT behavior
  bool reset_previous_data_each_iteration{true};
  bool enable_debug_info{false};

  // Trajectory parameters (passed to MPT)
  double output_delta_arc_length_m{1.0};
  double output_backward_traj_length_m{5.0};

  // Ego nearest parameters (passed to MPT)
  double ego_nearest_dist_threshold_m{3.0};
  double ego_nearest_yaw_threshold_deg{45.0};

  // Acceleration smoothing
  int acceleration_moving_average_window{
    5};  // Moving average window size for acceleration smoothing
};

class TrajectoryMPTOptimizer : public TrajectoryOptimizerPluginBase
{
public:
  TrajectoryMPTOptimizer() = default;

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
  // Core MPT optimizer instance
  std::shared_ptr<MPTOptimizer> mpt_optimizer_ptr_;

  // Debug visualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;

  // Vehicle information
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // Parameter structs
  MPTParams mpt_params_;
  EgoNearestParam ego_nearest_param_;
  TrajectoryParam traj_param_;
  std::shared_ptr<DebugData> debug_data_ptr_;
  std::shared_ptr<autoware_utils::TimeKeeper> mpt_time_keeper_;

  PlannerData create_planner_data(
    const TrajectoryPoints & traj_points, const trajectory_mpt_optimizer_utils::BoundsPair & bounds,
    const TrajectoryOptimizerData & data) const;

  void publish_debug_markers(
    const trajectory_mpt_optimizer_utils::BoundsPair & bounds,
    const TrajectoryPoints & traj_points) const;
};

}  // namespace autoware::trajectory_optimizer::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_MPT_OPTIMIZER_HPP_
