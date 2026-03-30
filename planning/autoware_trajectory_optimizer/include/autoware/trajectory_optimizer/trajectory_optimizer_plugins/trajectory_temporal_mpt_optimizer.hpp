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

#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_TEMPORAL_MPT_OPTIMIZER_HPP_  // NOLINT
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_TEMPORAL_MPT_OPTIMIZER_HPP_  // NOLINT

#include "acados_interface.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

struct TemporalMPTParams
{
  /** Kinematic bicycle CG distances [m]; fixed 1.0 m to match Python examples /
   * ocp.parameter_values. */
  double lf{1.0};
  double lr{1.0};
  size_t min_points_for_optimization{2};
  bool enable_debug_info{false};
  bool publish_debug_topics{true};
  bool write_replay_fixture{true};
  std::string replay_fixture_directory;
  bool log_replay_fixture_to_console{false};
};

class TrajectoryTemporalMPTOptimizer : public TrajectoryOptimizerPluginBase
{
public:
  TrajectoryTemporalMPTOptimizer() = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper) override;

  void optimize_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
    TrajectoryOptimizerData & data) override;

  void set_up_params() override;

  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  std::unique_ptr<temporal_mpt::AcadosInterface> acados_interface_;
  TemporalMPTParams mpt_params_;

  void create_or_reset_solver();
  void write_temporal_mpt_replay_fixture(
    const std::array<double, temporal_mpt::NX> & x0, const TrajectoryPoints & reference_trajectory,
    int acados_status, const char * tag);
  void ensure_debug_publishers();
  void publish_temporal_mpt_debug_io(
    const TrajectoryPoints & reference_before, const nav_msgs::msg::Odometry & initial_odom,
    const TrajectoryPoints & trajectory_after, size_t output_point_count, int acados_status,
    const temporal_mpt::AcadosSolution * mpc_solution);

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr debug_input_trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr debug_input_initial_state_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    debug_output_trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr debug_solve_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_control_accel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_control_delta_cmd_pub_;
};

}  // namespace autoware::trajectory_optimizer::plugin
// clang-format off
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_TEMPORAL_MPT_OPTIMIZER_HPP_  // NOLINT
// clang-format on
