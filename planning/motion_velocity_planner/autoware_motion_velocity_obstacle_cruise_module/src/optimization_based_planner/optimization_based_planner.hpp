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

#ifndef OPTIMIZATION_BASED_PLANNER__OPTIMIZATION_BASED_PLANNER_HPP_  // NOLINT
#define OPTIMIZATION_BASED_PLANNER__OPTIMIZATION_BASED_PLANNER_HPP_  // NOLINT

#include "../cruise_planner_interface.hpp"
#include "../type_alias.hpp"
#include "../types.hpp"
#include "s_boundary.hpp"
#include "velocity_optimizer.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>
#include <tuple>
#include <vector>

namespace autoware::motion_velocity_planner
{
class OptimizationBasedPlanner : public CruisePlannerInterface
{
public:
  OptimizationBasedPlanner(
    rclcpp::Node & node, const CommonParam & common_param,
    const CruisePlanningParam & cruise_planning_param);

  std::vector<TrajectoryPoint> plan_cruise(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & stop_traj_points,
    const std::vector<CruiseObstacle> & obstacles, std::shared_ptr<DebugData> debug_data_ptr,
    std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface> &
      planning_factor_interface,
    std::optional<VelocityLimit> & velocity_limit) override;

  void update_parameters(
    [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters) override
  {
  }

private:
  // Member Functions
  std::vector<double> createTimeVector();
  std::tuple<double, double> calcInitialMotion(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & stop_traj_points,
    const std::vector<TrajectoryPoint> & prev_traj_points);

  static bool checkHasReachedGoal(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & stop_traj_points);

  std::optional<SBoundaries> getSBoundaries(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & stop_traj_points,
    const std::vector<CruiseObstacle> & obstacles, const std::vector<double> & time_vec);

  std::optional<SBoundaries> getSBoundaries(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & stop_traj_points, const CruiseObstacle & object,
    const std::vector<double> & time_vec, const double traj_length);

  std::optional<SBoundaries> getSBoundariesForOnTrajectoryObject(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::shared_ptr<const PlannerData> planner_data, const std::vector<double> & time_vec,
    const double safety_distance, const CruiseObstacle & object, const double traj_length) const;

  std::optional<SBoundaries> getSBoundariesForOffTrajectoryObject(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::shared_ptr<const PlannerData> planner_data, const std::vector<double> & time_vec,
    const double safety_distance, const CruiseObstacle & object, const double traj_length);

  bool checkOnTrajectory(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & stop_traj_points,
    const polygon_utils::PointWithStamp & point);

  static std::optional<double> calcTrajectoryLengthFromCurrentPose(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & ego_pose,
    const std::shared_ptr<const PlannerData> planner_data);

  geometry_msgs::msg::Pose transformBaseLink2Center(
    const geometry_msgs::msg::Pose & pose_base_link);

  static std::optional<VelocityOptimizer::OptimizationResult> processOptimizedResult(
    const double v0, const VelocityOptimizer::OptimizationResult & opt_result, const double offset);

  void publishDebugTrajectory(
    [[maybe_unused]] const std::shared_ptr<const PlannerData> planner_data, const double offset,
    const std::vector<double> & time_vec, const SBoundaries & s_boundaries,
    const VelocityOptimizer::OptimizationResult & opt_result);

  std::vector<TrajectoryPoint> prev_output_;

  // Velocity Optimizer
  std::shared_ptr<VelocityOptimizer> velocity_optimizer_ptr_;

  // Publisher
  rclcpp::Publisher<Trajectory>::SharedPtr boundary_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr optimized_sv_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr optimized_st_graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_wall_marker_pub_;

  // Subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr smoothed_traj_sub_;

  Trajectory::ConstSharedPtr smoothed_trajectory_ptr_{nullptr};

  // Resampling Parameter
  double dense_resampling_time_interval_;
  double sparse_resampling_time_interval_;
  double dense_time_horizon_;
  double max_time_horizon_;

  double t_dangerous_;
  double velocity_margin_;

  double replan_vel_deviation_;
  double engage_velocity_;
  double engage_acceleration_;
  double engage_exit_ratio_;
  double stop_dist_to_prohibit_engage_;
};
}  // namespace autoware::motion_velocity_planner
// clang-format off
#endif  // OPTIMIZATION_BASED_PLANNER__OPTIMIZATION_BASED_PLANNER_HPP_  // NOLINT
// clang-format on
