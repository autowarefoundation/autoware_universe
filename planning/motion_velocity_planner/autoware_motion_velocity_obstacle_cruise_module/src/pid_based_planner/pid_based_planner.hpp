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

#ifndef PID_BASED_PLANNER__PID_BASED_PLANNER_HPP_
#define PID_BASED_PLANNER__PID_BASED_PLANNER_HPP_

#include "../cruise_planner_interface.hpp"
#include "../type_alias.hpp"
#include "../types.hpp"
#include "autoware/signal_processing/lowpass_filter_1d.hpp"
#include "cruise_planning_debug_info.hpp"
#include "pid_controller.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>
#include <optional>
#include <vector>

using autoware::signal_processing::LowpassFilter1d;

namespace autoware::motion_velocity_planner
{

class PIDBasedPlanner : public CruisePlannerInterface
{
public:
  struct CruiseObstacleInfo
  {
    CruiseObstacleInfo(
      const CruiseObstacle & obstacle_arg, const double error_cruise_dist_arg,
      const double dist_to_obstacle_arg, const double target_dist_to_obstacle_arg)
    : obstacle(obstacle_arg),
      error_cruise_dist(error_cruise_dist_arg),
      dist_to_obstacle(dist_to_obstacle_arg),
      target_dist_to_obstacle(target_dist_to_obstacle_arg)
    {
    }

    CruiseObstacle obstacle;
    double error_cruise_dist;
    double dist_to_obstacle;
    double target_dist_to_obstacle;
  };

  PIDBasedPlanner(
    rclcpp::Node & node, const CommonParam & common_param,
    const CruisePlanningParam & cruise_planning_param);

  std::vector<TrajectoryPoint> plan_cruise(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & stop_traj_points,
    const std::vector<CruiseObstacle> & obstacles, std::shared_ptr<DebugData> debug_data_ptr,
    std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface> &
      planning_factor_interface,
    std::optional<VelocityLimit> & velocity_limit) override;

  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;

private:
  Float32MultiArrayStamped get_cruise_planning_debug_message(
    const rclcpp::Time & current_time) const override
  {
    return cruise_planning_debug_info_.convert_to_message(current_time);
  }

  std::optional<CruiseObstacleInfo> calc_obstacle_to_cruise(
    const std::vector<TrajectoryPoint> & stop_traj_points,
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<CruiseObstacle> & obstacles);

  std::vector<TrajectoryPoint> plan_cruise_trajectory(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & stop_traj_points,
    std::shared_ptr<DebugData> debug_data_ptr,
    std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface> &
      planning_factor_interface,
    std::optional<VelocityLimit> & velocity_limit,
    const std::optional<CruiseObstacleInfo> & cruise_obstacle_info);

  // velocity limit based planner
  VelocityLimit plan_cruise_with_velocity_limit(
    const std::shared_ptr<const PlannerData> planner_data,
    const CruiseObstacleInfo & cruise_obstacle_info);

  // velocity insertion based planner
  std::vector<TrajectoryPoint> plan_cruise_with_trajectory(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & stop_traj_points,
    const CruiseObstacleInfo & cruise_obstacle_info);
  std::vector<TrajectoryPoint> get_acceleration_limited_trajectory(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & start_pose,
    const double v0, const double a0, const double target_acc, const double target_jerk_ratio,
    const std::shared_ptr<const PlannerData> planner_data) const;

  // ROS parameters
  double min_accel_during_cruise_;
  double min_cruise_target_vel_;

  CruisePlanningDebugInfo cruise_planning_debug_info_;

  struct VelocityLimitBasedPlannerParam
  {
    std::unique_ptr<PIDController> pid_vel_controller;
    double output_ratio_during_accel;
    double vel_to_acc_weight;
    bool enable_jerk_limit_to_output_acc{false};
    bool disable_target_acceleration{false};
  };
  VelocityLimitBasedPlannerParam velocity_limit_based_planner_param_;

  struct VelocityInsertionBasedPlannerParam
  {
    std::unique_ptr<PIDController> pid_acc_controller;
    std::unique_ptr<PIDController> pid_jerk_controller;
    double output_acc_ratio_during_accel;
    double output_jerk_ratio_during_accel;
    bool enable_jerk_limit_to_output_acc{false};
  };
  VelocityInsertionBasedPlannerParam velocity_insertion_based_planner_param_;

  std::optional<double> prev_target_acc_;

  // lpf for nodes's input
  std::shared_ptr<LowpassFilter1d> lpf_dist_to_obstacle_ptr_;
  std::shared_ptr<LowpassFilter1d> lpf_error_cruise_dist_ptr_;
  std::shared_ptr<LowpassFilter1d> lpf_obstacle_vel_ptr_;

  // lpf for planner's input
  std::shared_ptr<LowpassFilter1d> lpf_normalized_error_cruise_dist_ptr_;

  // lpf for output
  std::shared_ptr<LowpassFilter1d> lpf_output_vel_ptr_;
  std::shared_ptr<LowpassFilter1d> lpf_output_acc_ptr_;
  std::shared_ptr<LowpassFilter1d> lpf_output_jerk_ptr_;

  std::vector<TrajectoryPoint> prev_traj_points_;

  bool use_velocity_limit_based_planner_{true};

  double time_to_evaluate_rss_;

  std::function<double(double)> error_func_;
};
}  // namespace autoware::motion_velocity_planner

#endif  // PID_BASED_PLANNER__PID_BASED_PLANNER_HPP_
