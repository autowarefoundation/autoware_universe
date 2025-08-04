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

#include "pid_based_planner.hpp"

#include "autoware/interpolation/spline_interpolation.hpp"
#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/motion_velocity_planner_common/utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/ros/marker_helper.hpp"
#include "autoware_utils/ros/update_param.hpp"

#include "autoware_internal_planning_msgs/msg/velocity_limit.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using autoware::signal_processing::LowpassFilter1d;
namespace autoware::motion_velocity_planner
{

namespace
{
VelocityLimit create_velocity_limit_message(
  const rclcpp::Time & current_time, const double vel, const double acc, const double max_jerk,
  const double min_jerk)
{
  VelocityLimit msg;
  msg.stamp = current_time;
  msg.sender = "obstacle_cruise";
  msg.use_constraints = true;

  msg.max_velocity = vel;
  if (acc < 0) {
    msg.constraints.min_acceleration = acc;
  }
  msg.constraints.max_jerk = max_jerk;
  msg.constraints.min_jerk = min_jerk;

  return msg;
}

template <typename T>
T getSign(const T val)
{
  if (0 < val) {
    return static_cast<T>(1);
  } else if (val < 0) {
    return static_cast<T>(-1);
  }
  return static_cast<T>(0);
}
}  // namespace

PIDBasedPlanner::PIDBasedPlanner(
  rclcpp::Node & node, const CommonParam & common_param,
  const CruisePlanningParam & cruise_planning_param)
: CruisePlannerInterface(node, common_param, cruise_planning_param)
{
  min_accel_during_cruise_ = node.declare_parameter<double>(
    "obstacle_cruise.cruise_planning.pid_based_planner.min_accel_during_cruise");

  use_velocity_limit_based_planner_ = node.declare_parameter<bool>(
    "obstacle_cruise.cruise_planning.pid_based_planner.use_velocity_limit_based_planner");

  {  // velocity limit based planner
    const double kp = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.kp");
    const double ki = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.ki");
    const double kd = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.kd");
    velocity_limit_based_planner_param_.pid_vel_controller =
      std::make_unique<PIDController>(kp, ki, kd);

    velocity_limit_based_planner_param_.output_ratio_during_accel = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.output_ratio_"
      "during_accel");
    velocity_limit_based_planner_param_.vel_to_acc_weight = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.vel_to_acc_"
      "weight");

    velocity_limit_based_planner_param_.enable_jerk_limit_to_output_acc =
      node.declare_parameter<bool>(
        "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.enable_"
        "jerk_limit_to_output_acc");

    velocity_limit_based_planner_param_.disable_target_acceleration = node.declare_parameter<bool>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.disable_"
      "target_acceleration");
  }

  {  // velocity insertion based planner
    // pid controller for acc
    const double kp_acc = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kp_acc");
    const double ki_acc = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.ki_acc");
    const double kd_acc = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kd_acc");
    velocity_insertion_based_planner_param_.pid_acc_controller =
      std::make_unique<PIDController>(kp_acc, ki_acc, kd_acc);

    // pid controller for jerk
    const double kp_jerk = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kp_jerk");
    const double ki_jerk = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.ki_jerk");
    const double kd_jerk = node.declare_parameter<double>(
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kd_jerk");
    velocity_insertion_based_planner_param_.pid_jerk_controller =
      std::make_unique<PIDController>(kp_jerk, ki_jerk, kd_jerk);

    velocity_insertion_based_planner_param_.output_acc_ratio_during_accel =
      node.declare_parameter<double>(
        "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.output_"
        "acc_ratio_during_accel");
    velocity_insertion_based_planner_param_.output_jerk_ratio_during_accel =
      node.declare_parameter<double>(
        "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.output_"
        "jerk_ratio_during_accel");

    velocity_insertion_based_planner_param_.enable_jerk_limit_to_output_acc =
      node.declare_parameter<bool>(
        "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.enable_"
        "jerk_limit_to_output_acc");
  }

  min_cruise_target_vel_ = node.declare_parameter<double>(
    "obstacle_cruise.cruise_planning.pid_based_planner.min_cruise_target_vel");
  time_to_evaluate_rss_ = node.declare_parameter<double>(
    "obstacle_cruise.cruise_planning.pid_based_planner.time_to_evaluate_rss");
  const auto error_function_type = node.declare_parameter<std::string>(
    "obstacle_cruise.cruise_planning.pid_based_planner.error_function_type");
  error_func_ = [&]() -> std::function<double(double)> {
    if (error_function_type == "linear") {
      return [](const double val) { return val; };
    } else if (error_function_type == "quadratic") {
      return [](const double val) { return getSign(val) * std::pow(val, 2); };
    }
    throw std::domain_error("error function type is not supported");
  }();

  // low pass filter
  const double lpf_normalized_error_cruise_dist_gain = node.declare_parameter<double>(
    "obstacle_cruise.cruise_planning.pid_based_planner.lpf_normalized_error_cruise_dist_gain");
  lpf_normalized_error_cruise_dist_ptr_ =
    std::make_shared<LowpassFilter1d>(lpf_normalized_error_cruise_dist_gain);
  lpf_dist_to_obstacle_ptr_ = std::make_shared<LowpassFilter1d>(0.5);
  lpf_obstacle_vel_ptr_ = std::make_shared<LowpassFilter1d>(0.5);
  lpf_error_cruise_dist_ptr_ = std::make_shared<LowpassFilter1d>(0.5);
  lpf_output_vel_ptr_ = std::make_shared<LowpassFilter1d>(0.5);
  lpf_output_acc_ptr_ = std::make_shared<LowpassFilter1d>(0.5);
  lpf_output_jerk_ptr_ = std::make_shared<LowpassFilter1d>(0.5);
}

std::vector<TrajectoryPoint> PIDBasedPlanner::plan_cruise(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & stop_traj_points,
  const std::vector<CruiseObstacle> & obstacles, std::shared_ptr<DebugData> debug_data_ptr,
  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface> &
    planning_factor_interface,
  std::optional<VelocityLimit> & velocity_limit)
{
  cruise_planning_debug_info_.reset();

  // calc obstacles to cruise
  const auto cruise_obstacle_info =
    calc_obstacle_to_cruise(stop_traj_points, planner_data, obstacles);

  // plan cruise
  const auto cruise_traj_points = plan_cruise_trajectory(
    planner_data, stop_traj_points, debug_data_ptr, planning_factor_interface, velocity_limit,
    cruise_obstacle_info);

  prev_traj_points_ = cruise_traj_points;
  return cruise_traj_points;
}

std::optional<PIDBasedPlanner::CruiseObstacleInfo> PIDBasedPlanner::calc_obstacle_to_cruise(
  const std::vector<TrajectoryPoint> & stop_traj_points,
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<CruiseObstacle> & obstacles)
{
  cruise_planning_debug_info_.set(
    CruisePlanningDebugInfo::TYPE::EGO_VELOCITY,
    planner_data->current_odometry.twist.twist.linear.x);
  cruise_planning_debug_info_.set(
    CruisePlanningDebugInfo::TYPE::EGO_ACCELERATION,
    planner_data->current_acceleration.accel.accel.linear.x);

  auto modified_target_obstacles = obstacles;  // TODO(murooka) what is this variable?

  // search highest probability obstacle for cruise
  std::optional<CruiseObstacleInfo> cruise_obstacle_info;
  for (size_t o_idx = 0; o_idx < obstacles.size(); ++o_idx) {
    const auto & obstacle = obstacles.at(o_idx);

    if (obstacle.collision_points.empty()) {
      continue;
    }

    // NOTE: from ego's front to obstacle's back
    // TODO(murooka) this is not dist to obstacle
    const double dist_to_obstacle =
      calc_distance_to_collisionPoint(
        stop_traj_points, planner_data, obstacle.collision_points.front().point) +
      (obstacle.velocity - planner_data->current_odometry.twist.twist.linear.x) *
        time_to_evaluate_rss_;

    // calculate distance between ego and obstacle based on RSS
    const double target_dist_to_obstacle = calc_rss_distance(
      planner_data->current_odometry.twist.twist.linear.x, obstacle.velocity,
      cruise_planning_param_.safe_distance_margin);

    // calculate error distance and normalized one
    const double error_cruise_dist = dist_to_obstacle - target_dist_to_obstacle;
    if (cruise_obstacle_info) {
      if (error_cruise_dist > cruise_obstacle_info->error_cruise_dist) {
        continue;
      }
    }
    cruise_obstacle_info =
      CruiseObstacleInfo(obstacle, error_cruise_dist, dist_to_obstacle, target_dist_to_obstacle);
  }

  if (!cruise_obstacle_info) {  // if obstacle to cruise was not found
    lpf_dist_to_obstacle_ptr_->reset();
    lpf_obstacle_vel_ptr_->reset();
    lpf_error_cruise_dist_ptr_->reset();
    return {};
  }

  // if obstacle to cruise was found
  {  // debug data
    cruise_planning_debug_info_.set(
      CruisePlanningDebugInfo::TYPE::CRUISE_RELATIVE_EGO_VELOCITY,
      planner_data->current_odometry.twist.twist.linear.x -
        cruise_obstacle_info->obstacle.velocity);
    const double non_zero_relative_vel = [&]() {
      const double relative_vel = planner_data->current_odometry.twist.twist.linear.x -
                                  cruise_obstacle_info->obstacle.velocity;
      constexpr double epsilon = 0.001;
      if (epsilon < std::abs(relative_vel)) {
        return relative_vel;
      }
      return getSign(relative_vel) * epsilon;
    }();
    cruise_planning_debug_info_.set(
      CruisePlanningDebugInfo::TYPE::CRUISE_TIME_TO_COLLISION,
      cruise_obstacle_info->dist_to_obstacle / non_zero_relative_vel);
  }

  {  // dist to obstacle
    const double raw_dist_to_obstacle = cruise_obstacle_info->dist_to_obstacle;
    const double filtered_dist_to_obstacle =
      lpf_dist_to_obstacle_ptr_->filter(cruise_obstacle_info->dist_to_obstacle);

    cruise_planning_debug_info_.set(
      CruisePlanningDebugInfo::TYPE::CRUISE_CURRENT_OBSTACLE_DISTANCE_RAW, raw_dist_to_obstacle);
    cruise_planning_debug_info_.set(
      CruisePlanningDebugInfo::TYPE::CRUISE_CURRENT_OBSTACLE_DISTANCE_FILTERED,
      filtered_dist_to_obstacle);

    cruise_obstacle_info->dist_to_obstacle = filtered_dist_to_obstacle;
  }

  {  // obstacle velocity
    const double raw_obstacle_velocity = cruise_obstacle_info->obstacle.velocity;
    const double filtered_obstacle_velocity = lpf_obstacle_vel_ptr_->filter(raw_obstacle_velocity);

    cruise_planning_debug_info_.set(
      CruisePlanningDebugInfo::TYPE::CRUISE_CURRENT_OBSTACLE_VELOCITY_RAW, raw_obstacle_velocity);
    cruise_planning_debug_info_.set(
      CruisePlanningDebugInfo::TYPE::CRUISE_CURRENT_OBSTACLE_VELOCITY_FILTERED,
      filtered_obstacle_velocity);

    cruise_obstacle_info->obstacle.velocity = filtered_obstacle_velocity;
  }

  {  // error distance for cruising
    const double raw_error_cruise_dist = cruise_obstacle_info->error_cruise_dist;
    const double filtered_error_cruise_dist =
      lpf_error_cruise_dist_ptr_->filter(cruise_obstacle_info->error_cruise_dist);

    cruise_planning_debug_info_.set(
      CruisePlanningDebugInfo::TYPE::CRUISE_ERROR_DISTANCE_RAW, raw_error_cruise_dist);
    cruise_planning_debug_info_.set(
      CruisePlanningDebugInfo::TYPE::CRUISE_ERROR_DISTANCE_FILTERED, filtered_error_cruise_dist);

    cruise_obstacle_info->error_cruise_dist = filtered_error_cruise_dist;
  }

  {  // target dist for cruising
    cruise_planning_debug_info_.set(
      CruisePlanningDebugInfo::TYPE::CRUISE_TARGET_OBSTACLE_DISTANCE,
      cruise_obstacle_info->target_dist_to_obstacle);
  }

  return cruise_obstacle_info;
}

std::vector<TrajectoryPoint> PIDBasedPlanner::plan_cruise_trajectory(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & stop_traj_points, std::shared_ptr<DebugData> debug_data_ptr,
  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface> &
    planning_factor_interface,
  std::optional<VelocityLimit> & velocity_limit,
  const std::optional<CruiseObstacleInfo> & cruise_obstacle_info)
{
  // do cruise
  if (cruise_obstacle_info) {
    RCLCPP_DEBUG(logger_, "cruise planning");

    {  // update debug marker
      // virtual wall marker for cruise
      const double error_cruise_dist = cruise_obstacle_info->error_cruise_dist;
      const double dist_to_obstacle = cruise_obstacle_info->dist_to_obstacle;
      const size_t ego_idx =
        planner_data->find_index(stop_traj_points, planner_data->current_odometry.pose.pose);
      const double abs_ego_offset =
        planner_data->is_driving_forward
          ? std::abs(planner_data->vehicle_info_.max_longitudinal_offset_m)
          : std::abs(planner_data->vehicle_info_.min_longitudinal_offset_m);
      const double dist_to_rss_wall =
        std::min(error_cruise_dist + abs_ego_offset, dist_to_obstacle + abs_ego_offset);
      const size_t wall_idx =
        utils::get_index_with_longitudinal_offset(stop_traj_points, dist_to_rss_wall, ego_idx);

      const std::string wall_reason_string = cruise_obstacle_info->obstacle.is_yield_obstacle
                                               ? "obstacle cruise (yield)"
                                               : "obstacle cruise";
      auto markers = autoware::motion_utils::createSlowDownVirtualWallMarker(
        stop_traj_points.at(wall_idx).pose, wall_reason_string, clock_->now(), 0);
      // NOTE: use a different color from slow down one to visualize cruise and slow down
      // separately.
      markers.markers.front().color = autoware_utils::create_marker_color(1.0, 0.6, 0.1, 0.5);
      autoware_utils::append_marker_array(markers, &debug_data_ptr->cruise_wall_marker);

      // cruise obstacle
      debug_data_ptr->obstacles_to_cruise.push_back(cruise_obstacle_info->obstacle);

      planning_factor_interface->add(
        stop_traj_points, planner_data->current_odometry.pose.pose,
        stop_traj_points.at(wall_idx).pose, PlanningFactor::NONE, SafetyFactorArray{});
    }

    // do cruise planning
    if (!use_velocity_limit_based_planner_) {
      const auto cruise_traj =
        plan_cruise_with_trajectory(planner_data, stop_traj_points, *cruise_obstacle_info);
      return cruise_traj;
    }

    velocity_limit = plan_cruise_with_velocity_limit(planner_data, *cruise_obstacle_info);
    return stop_traj_points;
  }

  // reset previous cruise data if adaptive cruise is not enabled
  prev_target_acc_ = {};
  lpf_normalized_error_cruise_dist_ptr_->reset();
  lpf_output_acc_ptr_->reset();
  lpf_output_vel_ptr_->reset();
  lpf_output_jerk_ptr_->reset();

  // delete marker
  const auto markers =
    autoware::motion_utils::createDeletedSlowDownVirtualWallMarker(clock_->now(), 0);
  autoware_utils::append_marker_array(markers, &debug_data_ptr->cruise_wall_marker);

  return stop_traj_points;
}

VelocityLimit PIDBasedPlanner::plan_cruise_with_velocity_limit(
  const std::shared_ptr<const PlannerData> planner_data,
  const CruiseObstacleInfo & cruise_obstacle_info)
{
  const auto & p = velocity_limit_based_planner_param_;

  const double filtered_normalized_error_cruise_dist = [&]() {
    const double normalized_error_cruise_dist =
      cruise_obstacle_info.error_cruise_dist / cruise_obstacle_info.dist_to_obstacle;
    return lpf_normalized_error_cruise_dist_ptr_->filter(normalized_error_cruise_dist);
  }();

  const double modified_error_cruise_dist = error_func_(filtered_normalized_error_cruise_dist);

  // calculate target velocity with acceleration limit by PID controller
  const double pid_output_vel = p.pid_vel_controller->calc(modified_error_cruise_dist);
  const double additional_vel = [&]() {
    if (modified_error_cruise_dist > 0) {
      return pid_output_vel * p.output_ratio_during_accel;
    }
    return pid_output_vel;
  }();

  const double positive_target_vel = lpf_output_vel_ptr_->filter(
    std::max(
      min_cruise_target_vel_,
      planner_data->current_odometry.twist.twist.linear.x + additional_vel));

  // calculate target acceleration
  const double target_acc = [&]() {
    if (p.disable_target_acceleration) {
      return min_accel_during_cruise_;
    }

    double target_acc = p.vel_to_acc_weight * additional_vel;

    // apply acc limit
    target_acc =
      std::clamp(target_acc, min_accel_during_cruise_, common_param_.max_accel);  // apply acc limit

    if (!prev_target_acc_) {
      return lpf_output_acc_ptr_->filter(target_acc);
    }

    if (p.enable_jerk_limit_to_output_acc) {                       // apply jerk limit
      const double jerk = (target_acc - *prev_target_acc_) / 0.1;  // TODO(murooka) 0.1
      const double limited_jerk = std::clamp(jerk, common_param_.min_jerk, common_param_.max_jerk);
      target_acc = *prev_target_acc_ + limited_jerk * 0.1;
    }

    return lpf_output_acc_ptr_->filter(target_acc);
  }();
  prev_target_acc_ = target_acc;

  cruise_planning_debug_info_.set(
    CruisePlanningDebugInfo::TYPE::CRUISE_TARGET_VELOCITY, positive_target_vel);
  cruise_planning_debug_info_.set(
    CruisePlanningDebugInfo::TYPE::CRUISE_TARGET_ACCELERATION, target_acc);

  RCLCPP_DEBUG(logger_, "target_velocity %f", positive_target_vel);

  // set target longitudinal motion
  const auto velocity_limit = create_velocity_limit_message(
    clock_->now(), positive_target_vel, target_acc, common_param_.max_jerk, common_param_.min_jerk);

  return velocity_limit;
}

std::vector<TrajectoryPoint> PIDBasedPlanner::plan_cruise_with_trajectory(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & stop_traj_points,
  const CruiseObstacleInfo & cruise_obstacle_info)
{
  const auto & p = velocity_insertion_based_planner_param_;

  const double filtered_normalized_error_cruise_dist = [&]() {
    const double normalized_error_cruise_dist =
      cruise_obstacle_info.error_cruise_dist / cruise_obstacle_info.dist_to_obstacle;
    return lpf_normalized_error_cruise_dist_ptr_->filter(normalized_error_cruise_dist);
  }();

  const double modified_error_cruise_dist = error_func_(filtered_normalized_error_cruise_dist);

  // calculate target velocity with acceleration limit by PID controller
  // calculate target acceleration
  const double target_acc = [&]() {
    double target_acc = p.pid_acc_controller->calc(modified_error_cruise_dist);

    if (0 < filtered_normalized_error_cruise_dist) {
      target_acc *= p.output_acc_ratio_during_accel;
    }

    target_acc =
      std::clamp(target_acc, min_accel_during_cruise_, common_param_.max_accel);  // apply acc limit

    if (!prev_target_acc_) {
      return target_acc;
    }

    if (p.enable_jerk_limit_to_output_acc) {
      const double jerk = (target_acc - *prev_target_acc_) / 0.1;  // TODO(murooka) 0.1
      const double limited_jerk = std::clamp(jerk, common_param_.min_jerk, common_param_.max_jerk);
      target_acc = *prev_target_acc_ + limited_jerk * 0.1;
    }

    return target_acc;
  }();
  prev_target_acc_ = target_acc;

  const double target_jerk_ratio = [&]() {
    double target_jerk_ratio = p.pid_jerk_controller->calc(modified_error_cruise_dist);

    if (0 < filtered_normalized_error_cruise_dist) {
      target_jerk_ratio *= p.output_jerk_ratio_during_accel;
    }

    target_jerk_ratio = std::clamp(std::abs(target_jerk_ratio), 0.0, 1.0);
    return target_jerk_ratio;
  }();

  cruise_planning_debug_info_.set(
    CruisePlanningDebugInfo::TYPE::CRUISE_TARGET_ACCELERATION, target_acc);
  cruise_planning_debug_info_.set(
    CruisePlanningDebugInfo::TYPE::CRUISE_TARGET_JERK_RATIO, target_jerk_ratio);

  // set target longitudinal motion
  const auto prev_traj_closest_point = [&]() -> TrajectoryPoint {
    // if (smoothed_trajectory_ptr_) {
    //   return autoware::motion_utils::calcInterpolatedPoint(
    //     *smoothed_trajectory_ptr_, planner_data->current_odometry.pose.pose,
    //     nearest_dist_deviation_threshold_, nearest_yaw_deviation_threshold_);
    // }
    return autoware::motion_utils::calcInterpolatedPoint(
      autoware::motion_utils::convertToTrajectory(prev_traj_points_),
      planner_data->current_odometry.pose.pose, planner_data->ego_nearest_dist_threshold,
      planner_data->ego_nearest_yaw_threshold);
  }();
  const double v0 = prev_traj_closest_point.longitudinal_velocity_mps;
  const double a0 = prev_traj_closest_point.acceleration_mps2;

  auto cruise_traj_points = get_acceleration_limited_trajectory(
    stop_traj_points, planner_data->current_odometry.pose.pose, v0, a0, target_acc,
    target_jerk_ratio, planner_data);

  const auto zero_vel_idx_opt = autoware::motion_utils::searchZeroVelocityIndex(cruise_traj_points);
  if (!zero_vel_idx_opt) {
    return cruise_traj_points;
  }

  for (size_t i = zero_vel_idx_opt.value(); i < cruise_traj_points.size(); ++i) {
    cruise_traj_points.at(i).longitudinal_velocity_mps = 0.0;
  }

  return cruise_traj_points;
}

std::vector<TrajectoryPoint> PIDBasedPlanner::get_acceleration_limited_trajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & start_pose,
  const double v0, const double a0, const double target_acc, const double target_jerk_ratio,
  const std::shared_ptr<const PlannerData> planner_data) const
{
  // calculate vt function
  const auto vt = [&](
                    const double v0, const double a0, const double jerk, const double t,
                    const double target_acc) {
    const double t1 = (target_acc - a0) / jerk;

    const double v = [&]() {
      if (t < t1) {
        return v0 + a0 * t + jerk * t * t / 2.0;
      }

      const double v1 = v0 + a0 * t1 + jerk * t1 * t1 / 2.0;
      return v1 + target_acc * (t - t1);
    }();

    constexpr double max_vel = 100.0;
    return std::clamp(v, 0.0, max_vel);
  };

  // calculate sv graph
  const double s_traj = autoware::motion_utils::calcArcLength(traj_points);
  // const double t_max = 10.0;
  const double s_max = 50.0;
  const double max_sampling_num = 100.0;

  const double t_delta_min = 0.1;
  const double s_delta_min = 0.1;

  // NOTE: v0 of motion_velocity_planner_common may be smaller than v0 of motion_velocity_smoother
  //       Therefore, we calculate v1 (velocity at next step) and use it for initial velocity.
  const double v1 = v0;  // + 0.1;  // a0 * 0.1;  //  + jerk * 0.1 * 0.1 / 2.0;
  const double a1 = a0;  //  + jerk * 0.1;
  const double jerk = target_acc > a1 ? common_param_.max_jerk * target_jerk_ratio
                                      : common_param_.min_jerk * target_jerk_ratio;

  double t_current = 0.0;
  std::vector<double> s_vec{0.0};
  std::vector<double> v_vec{v1};
  for (double tmp_idx = 0.0; tmp_idx < max_sampling_num; ++tmp_idx) {
    if (v_vec.back() <= 0.0) {
      s_vec.push_back(s_vec.back() + s_delta_min);
      v_vec.push_back(0.0);
    } else {
      const double v_current = vt(
        v1, a1, jerk, t_current + t_delta_min,
        target_acc);  // TODO(murooka) + t_delta_min is not required

      const double s_delta = std::max(s_delta_min, v_current * t_delta_min);
      const double s_current = s_vec.back() + s_delta;

      s_vec.push_back(s_current);
      v_vec.push_back(v_current);

      const double t_delta = [&]() {
        if (v_current <= 0) {
          return 0.0;
        }

        constexpr double t_delta_max = 100.0;  // NOTE: to avoid computation explosion
        return std::min(t_delta_max, s_delta / v_current);
      }();

      t_current += t_delta;
    }

    if (s_traj < s_vec.back() /*|| t_max < t_current*/ || s_max < s_vec.back()) {
      break;
    }
  }

  std::vector<double> unique_s_vec{s_vec.front()};
  std::vector<double> unique_v_vec{v_vec.front()};
  for (size_t i = 0; i < s_vec.size(); ++i) {
    if (s_vec.at(i) == unique_s_vec.back()) {
      continue;
    }

    unique_s_vec.push_back(s_vec.at(i));
    unique_v_vec.push_back(v_vec.at(i));
  }

  if (unique_s_vec.size() < 2) {
    return traj_points;
  }

  auto acc_limited_traj_points = traj_points;
  const size_t ego_seg_idx = planner_data->find_index(acc_limited_traj_points, start_pose);
  double sum_dist = 0.0;
  for (size_t i = ego_seg_idx; i < acc_limited_traj_points.size(); ++i) {
    if (i != ego_seg_idx) {
      sum_dist += autoware_utils::calc_distance2d(
        acc_limited_traj_points.at(i - 1), acc_limited_traj_points.at(i));
    }

    const double vel = [&]() {
      if (unique_s_vec.back() < sum_dist) {
        return unique_v_vec.back();
      }
      return autoware::interpolation::spline(unique_s_vec, unique_v_vec, {sum_dist}).front();
    }();

    acc_limited_traj_points.at(i).longitudinal_velocity_mps = std::clamp(
      vel, 0.0,
      static_cast<double>(
        acc_limited_traj_points.at(i)
          .longitudinal_velocity_mps));  // TODO(murooka) this assumes forward driving
  }

  return acc_limited_traj_points;
}

void PIDBasedPlanner::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  autoware_utils::update_param<double>(
    parameters, "obstacle_cruise.cruise_planning.pid_based_planner.min_accel_during_cruise",
    min_accel_during_cruise_);

  {  // velocity limit based planner
    auto & p = velocity_limit_based_planner_param_;

    double kp = p.pid_vel_controller->getKp();
    double ki = p.pid_vel_controller->getKi();
    double kd = p.pid_vel_controller->getKd();

    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.kp", kp);
    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.ki", ki);
    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.kd", kd);
    p.pid_vel_controller->updateParam(kp, ki, kd);

    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.output_ratio_"
      "during_accel",
      p.output_ratio_during_accel);
    autoware_utils::update_param<double>(
      parameters, "obstacle_cruise.cruise_planning.pid_based_planner.vel_to_acc_weight",
      p.vel_to_acc_weight);

    autoware_utils::update_param<bool>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.enable_jerk_"
      "limit_to_output_acc",
      p.enable_jerk_limit_to_output_acc);

    autoware_utils::update_param<bool>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_limit_based_planner.disable_"
      "target_acceleration",
      p.disable_target_acceleration);
  }

  {  // velocity insertion based planner
    auto & p = velocity_insertion_based_planner_param_;

    // pid controller for acc
    double kp_acc = p.pid_acc_controller->getKp();
    double ki_acc = p.pid_acc_controller->getKi();
    double kd_acc = p.pid_acc_controller->getKd();

    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kp_acc",
      kp_acc);
    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.ki_acc",
      ki_acc);
    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kd_acc",
      kd_acc);
    p.pid_acc_controller->updateParam(kp_acc, ki_acc, kd_acc);

    // pid controller for jerk
    double kp_jerk = p.pid_jerk_controller->getKp();
    double ki_jerk = p.pid_jerk_controller->getKi();
    double kd_jerk = p.pid_jerk_controller->getKd();

    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kp_jerk",
      kp_jerk);
    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.ki_jerk",
      ki_jerk);
    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.kd_jerk",
      kd_jerk);
    p.pid_jerk_controller->updateParam(kp_jerk, ki_jerk, kd_jerk);

    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.output_"
      "acc_ratio_during_accel",
      p.output_acc_ratio_during_accel);
    autoware_utils::update_param<double>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.output_"
      "jerk_ratio_during_accel",
      p.output_jerk_ratio_during_accel);

    autoware_utils::update_param<bool>(
      parameters,
      "obstacle_cruise.cruise_planning.pid_based_planner.velocity_insertion_based_planner.enable_"
      "jerk_limit_to_output_acc",
      p.enable_jerk_limit_to_output_acc);
  }

  // min_cruise_target_vel
  autoware_utils::update_param<double>(
    parameters, "obstacle_cruise.cruise_planning.pid_based_planner.min_cruise_target_vel",
    min_cruise_target_vel_);
  autoware_utils::update_param<double>(
    parameters, "obstacle_cruise.cruise_planning.pid_based_planner.time_to_evaluate_rss",
    time_to_evaluate_rss_);
}
}  // namespace autoware::motion_velocity_planner
