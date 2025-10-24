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

#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_MPT_OPTIMIZER_HPP_  // NOLINT
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_MPT_OPTIMIZER_HPP_  // NOLINT

#include "autoware/path_optimizer/common_structs.hpp"
#include "autoware/path_optimizer/mpt_optimizer.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

struct BoundsPair
{
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;
};

struct TrajectoryMPTOptimizerParams
{
  double bounds_lateral_offset_m{2.0};
  bool enable_debug_info{false};
};

class TrajectoryMPTOptimizer : public TrajectoryOptimizerPluginBase
{
public:
  TrajectoryMPTOptimizer(
    const std::string name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const TrajectoryOptimizerParams & params);

  ~TrajectoryMPTOptimizer() = default;

  void optimize_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
    const TrajectoryOptimizerData & data) override;

  void set_up_params() override;

  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  // Map callback to receive lanelet map
  void on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);

  // Generate bounds perpendicular to trajectory heading at each point (fallback)
  BoundsPair generate_bounds_from_trajectory(
    const TrajectoryPoints & traj_points, double lateral_offset_m) const;

  // Generate bounds from lanelet map based on trajectory positions
  BoundsPair generate_bounds_from_lanelet_map(const TrajectoryPoints & traj_points) const;

  // Create PlannerData from trajectory points and generated bounds
  autoware::path_optimizer::PlannerData create_planner_data(
    const TrajectoryPoints & traj_points, const BoundsPair & bounds,
    const TrajectoryOptimizerData & data) const;

  // Publish debug markers for bounds visualization
  void publish_bounds_markers(const BoundsPair & bounds) const;

  // MPT optimizer instance
  std::shared_ptr<autoware::path_optimizer::MPTOptimizer> mpt_optimizer_ptr_;

  // Vehicle info
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // Plugin parameters
  TrajectoryMPTOptimizerParams mpt_params_;

  // Trajectory parameters (for MPT optimizer)
  autoware::path_optimizer::TrajectoryParam traj_param_;

  // Ego nearest parameters (for MPT optimizer)
  autoware::path_optimizer::EgoNearestParam ego_nearest_param_;

  // Debug data (for MPT optimizer)
  std::shared_ptr<autoware::path_optimizer::DebugData> debug_data_ptr_;

  // Lanelet map subscription and storage
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;

  // Debug marker publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
};

}  // namespace autoware::trajectory_optimizer::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_MPT_OPTIMIZER_HPP_
