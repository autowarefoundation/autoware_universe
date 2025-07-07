// Copyright 2023 TIER IV, Inc.
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

#include "autoware/path_optimizer/mpt_optimizer.hpp"

#include "acados_mpc/include/acados_interface.hpp"
#include "autoware/interpolation/spline_interpolation_points_2d.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/path_optimizer/utils/geometry_utils.hpp"
#include "autoware/path_optimizer/utils/trajectory_utils.hpp"

#include <autoware_utils/math/normalization.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::path_optimizer
{
namespace
{
std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByUniformCircle(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t circle_num,
  const double radius_ratio)
{
  const double lateral_offset =
    abs(vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  const double radius = std::hypot(
                          vehicle_info.vehicle_length_m / static_cast<double>(circle_num) / 2.0,
                          vehicle_info.vehicle_width_m / 2.0 + lateral_offset) *
                        radius_ratio;
  const std::vector<double> radiuses(circle_num, radius);

  const double unit_lon_length = vehicle_info.vehicle_length_m / static_cast<double>(circle_num);
  std::vector<double> longitudinal_offsets;
  for (size_t i = 0; i < circle_num; ++i) {
    longitudinal_offsets.push_back(
      unit_lon_length / 2.0 + unit_lon_length * i - vehicle_info.rear_overhang_m);
  }

  return {radiuses, longitudinal_offsets};
}

std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByBicycleModel(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t circle_num,
  const double rear_radius_ratio, const double front_radius_ratio)
{
  if (circle_num < 2) {
    throw std::invalid_argument("circle_num is less than 2.");
  }
  const double lateral_offset =
    abs(vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  // 1st circle (rear wheel)
  const double rear_radius =
    vehicle_info.vehicle_width_m / 2.0 + lateral_offset * rear_radius_ratio;
  const double rear_lon_offset = 0.0;

  // 2nd circle (front wheel)
  const double front_radius =
    std::hypot(
      vehicle_info.vehicle_length_m / static_cast<double>(circle_num) / 2.0,
      vehicle_info.vehicle_width_m / 2.0 + lateral_offset) *
    front_radius_ratio;

  const double unit_lon_length = vehicle_info.vehicle_length_m / static_cast<double>(circle_num);
  const double front_lon_offset =
    unit_lon_length / 2.0 + unit_lon_length * (circle_num - 1) - vehicle_info.rear_overhang_m;

  return {{rear_radius, front_radius}, {rear_lon_offset, front_lon_offset}};
}

std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByFittingUniformCircle(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t circle_num)
{
  if (circle_num < 2) {
    throw std::invalid_argument("circle_num is less than 2.");
  }
  const double lateral_offset =
    abs(vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  const double radius = vehicle_info.vehicle_width_m / 2.0 + lateral_offset;
  std::vector<double> radiuses(circle_num, radius);

  const double unit_lon_length =
    vehicle_info.vehicle_length_m / static_cast<double>(circle_num - 1);
  std::vector<double> longitudinal_offsets;
  for (size_t i = 0; i < circle_num; ++i) {
    longitudinal_offsets.push_back(unit_lon_length * i - vehicle_info.rear_overhang_m);
    radiuses.push_back(radius);
  }

  return {radiuses, longitudinal_offsets};
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd> extractBounds(
  const std::vector<ReferencePoint> & ref_points, const size_t l_idx, const double offset)
{
  Eigen::VectorXd ub_vec(ref_points.size());
  Eigen::VectorXd lb_vec(ref_points.size());
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ub_vec(i) = ref_points.at(i).bounds_on_constraints.at(l_idx).upper_bound + offset;
    lb_vec(i) = ref_points.at(i).bounds_on_constraints.at(l_idx).lower_bound - offset;
  }
  return {ub_vec, lb_vec};
}

std::vector<double> toStdVector(const Eigen::VectorXd & eigen_vec)
{
  return {eigen_vec.data(), eigen_vec.data() + eigen_vec.rows()};
}

bool isLeft(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & target_pos)
{
  const double base_theta = tf2::getYaw(pose.orientation);
  const double target_theta = autoware_utils::calc_azimuth_angle(pose.position, target_pos);
  const double diff_theta = autoware_utils::normalize_radian(target_theta - base_theta);
  return diff_theta > 0;
}

// NOTE: Regarding boundary's sign, left is positive, and right is negative
double calcLateralDistToBounds(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & bound,
  const double additional_offset, const bool is_left_bound = true)
{
  constexpr double max_lat_offset_for_left = 5.0;
  constexpr double min_lat_offset_for_left = -5.0;

  const double max_lat_offset = is_left_bound ? max_lat_offset_for_left : -max_lat_offset_for_left;
  const double min_lat_offset = is_left_bound ? min_lat_offset_for_left : -min_lat_offset_for_left;
  const auto max_lat_offset_point =
    autoware_utils::calc_offset_pose(pose, 0.0, max_lat_offset, 0.0).position;
  const auto min_lat_offset_point =
    autoware_utils::calc_offset_pose(pose, 0.0, min_lat_offset, 0.0).position;

  double closest_dist_to_bound = max_lat_offset;
  for (size_t i = 0; i < bound.size() - 1; ++i) {
    const auto intersect_point = autoware_utils::intersect(
      min_lat_offset_point, max_lat_offset_point, bound.at(i), bound.at(i + 1));
    if (intersect_point) {
      const bool is_point_left = isLeft(pose, *intersect_point);
      const double dist_to_bound =
        autoware_utils::calc_distance2d(pose.position, *intersect_point) *
        (is_point_left ? 1.0 : -1.0);

      // the bound which is closest to the centerline will be chosen
      const double tmp_dist_to_bound =
        is_left_bound ? dist_to_bound - additional_offset : dist_to_bound + additional_offset;
      if (std::abs(tmp_dist_to_bound) < std::abs(closest_dist_to_bound)) {
        closest_dist_to_bound = tmp_dist_to_bound;
      }
    }
  }

  return closest_dist_to_bound;
}
}  // namespace

MPTOptimizer::MPTParam::MPTParam(
  rclcpp::Node * node, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  {  // option
    steer_limit_constraint = node->declare_parameter<bool>("mpt.option.steer_limit_constraint");
    enable_warm_start = node->declare_parameter<bool>("mpt.option.enable_warm_start");
    enable_manual_warm_start = node->declare_parameter<bool>("mpt.option.enable_manual_warm_start");
    enable_optimization_validation =
      node->declare_parameter<bool>("mpt.option.enable_optimization_validation");
    mpt_visualize_sampling_num = node->declare_parameter<int>("mpt.option.visualize_sampling_num");
  }

  {  // common
    num_points = node->declare_parameter<int>("mpt.common.num_points");
    delta_arc_length = node->declare_parameter<double>("mpt.common.delta_arc_length");
  }

  use_acados = node->declare_parameter<bool>("mpt.use_acados");

  // kinematics
  max_steer_rad = vehicle_info.max_steer_angle_rad;

  // NOTE: By default, optimization_center_offset will be vehicle_info.wheel_base * 0.8
  //       The 0.8 scale is adopted as it performed the best.
  constexpr double default_wheel_base_ratio = 0.8;
  optimization_center_offset = node->declare_parameter<double>(
    "mpt.kinematics.optimization_center_offset",
    vehicle_info.wheel_base_m * default_wheel_base_ratio);

  {  // clearance
    hard_clearance_from_road =
      node->declare_parameter<double>("mpt.clearance.hard_clearance_from_road");
    soft_clearance_from_road =
      node->declare_parameter<double>("mpt.clearance.soft_clearance_from_road");
  }

  {  // weight
    soft_collision_free_weight =
      node->declare_parameter<double>("mpt.weight.soft_collision_free_weight");

    lat_error_weight = node->declare_parameter<double>("mpt.weight.lat_error_weight");
    yaw_error_weight = node->declare_parameter<double>("mpt.weight.yaw_error_weight");
    yaw_error_rate_weight = node->declare_parameter<double>("mpt.weight.yaw_error_rate_weight");
    steer_input_weight = node->declare_parameter<double>("mpt.weight.steer_input_weight");
    steer_rate_weight = node->declare_parameter<double>("mpt.weight.steer_rate_weight");

    terminal_lat_error_weight =
      node->declare_parameter<double>("mpt.weight.terminal_lat_error_weight");
    terminal_yaw_error_weight =
      node->declare_parameter<double>("mpt.weight.terminal_yaw_error_weight");
    goal_lat_error_weight = node->declare_parameter<double>("mpt.weight.goal_lat_error_weight");
    goal_yaw_error_weight = node->declare_parameter<double>("mpt.weight.goal_yaw_error_weight");
  }

  {  // avoidance
    max_longitudinal_margin_for_bound_violation =
      node->declare_parameter<double>("mpt.avoidance.max_longitudinal_margin_for_bound_violation");
    max_bound_fixing_time = node->declare_parameter<double>("mpt.avoidance.max_bound_fixing_time");
    max_avoidance_cost = node->declare_parameter<double>("mpt.avoidance.max_avoidance_cost");
    avoidance_cost_margin = node->declare_parameter<double>("mpt.avoidance.avoidance_cost_margin");
    avoidance_cost_band_length =
      node->declare_parameter<double>("mpt.avoidance.avoidance_cost_band_length");
    avoidance_cost_decrease_rate =
      node->declare_parameter<double>("mpt.avoidance.avoidance_cost_decrease_rate");
    min_drivable_width = node->declare_parameter<double>("mpt.avoidance.min_drivable_width");

    avoidance_lat_error_weight =
      node->declare_parameter<double>("mpt.avoidance.weight.lat_error_weight");
    avoidance_yaw_error_weight =
      node->declare_parameter<double>("mpt.avoidance.weight.yaw_error_weight");
    avoidance_steer_input_weight =
      node->declare_parameter<double>("mpt.avoidance.weight.steer_input_weight");
  }

  {  // collision free constraints
    l_inf_norm = node->declare_parameter<bool>("mpt.collision_free_constraints.option.l_inf_norm");
    soft_constraint =
      node->declare_parameter<bool>("mpt.collision_free_constraints.option.soft_constraint");
    hard_constraint =
      node->declare_parameter<bool>("mpt.collision_free_constraints.option.hard_constraint");
  }

  {  // vehicle_circles
    // NOTE: Vehicle shape for collision free constraints is considered as a set of circles
    vehicle_circles_method =
      node->declare_parameter<std::string>("mpt.collision_free_constraints.vehicle_circles.method");

    // uniform circles
    vehicle_circles_uniform_circle_num = node->declare_parameter<int>(
      "mpt.collision_free_constraints.vehicle_circles.uniform_circle.num");
    vehicle_circles_uniform_circle_radius_ratio = node->declare_parameter<double>(
      "mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio");

    // bicycle model
    vehicle_circles_bicycle_model_num = node->declare_parameter<int>(
      "mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_"
      "calculation");
    vehicle_circles_bicycle_model_rear_radius_ratio = node->declare_parameter<double>(
      "mpt.collision_free_constraints.vehicle_circles."
      "bicycle_model.rear_radius_ratio");
    vehicle_circles_bicycle_model_front_radius_ratio = node->declare_parameter<double>(
      "mpt.collision_free_constraints.vehicle_circles."
      "bicycle_model.front_radius_ratio");

    // fitting uniform circles
    vehicle_circles_fitting_uniform_circle_num = node->declare_parameter<int>(
      "mpt.collision_free_constraints.vehicle_circles.fitting_uniform_circle.num");
  }

  {  // validation
    max_validation_lat_error = node->declare_parameter<double>("mpt.validation.max_lat_error");
    max_validation_yaw_error = node->declare_parameter<double>("mpt.validation.max_yaw_error");
  }
}

void MPTOptimizer::MPTParam::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  {  // option
    update_param<bool>(parameters, "mpt.option.steer_limit_constraint", steer_limit_constraint);
    update_param<bool>(parameters, "mpt.option.enable_warm_start", enable_warm_start);
    update_param<bool>(parameters, "mpt.option.enable_manual_warm_start", enable_manual_warm_start);
    update_param<bool>(
      parameters, "mpt.option.enable_optimization_validation", enable_optimization_validation);
    update_param<int>(parameters, "mpt.option.visualize_sampling_num", mpt_visualize_sampling_num);
  }

  // common
  update_param<int>(parameters, "mpt.common.num_points", num_points);
  update_param<double>(parameters, "mpt.common.delta_arc_length", delta_arc_length);

  update_param<bool>(parameters, "mpt.use_acados", use_acados);

  // kinematics
  update_param<double>(
    parameters, "mpt.kinematics.optimization_center_offset", optimization_center_offset);

  // collision_free_constraints
  update_param<bool>(parameters, "mpt.collision_free_constraints.option.l_inf_norm", l_inf_norm);
  update_param<bool>(
    parameters, "mpt.collision_free_constraints.option.soft_constraint", soft_constraint);
  update_param<bool>(
    parameters, "mpt.collision_free_constraints.option.hard_constraint", hard_constraint);

  {  // vehicle_circles
    update_param<std::string>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.method", vehicle_circles_method);

    // uniform circles
    update_param<int>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.uniform_circle.num",
      vehicle_circles_uniform_circle_num);
    update_param<double>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio",
      vehicle_circles_uniform_circle_radius_ratio);

    // bicycle model
    update_param<int>(
      parameters,
      "mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_calculation",
      vehicle_circles_bicycle_model_num);
    update_param<double>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.bicycle_model.rear_radius_ratio",
      vehicle_circles_bicycle_model_rear_radius_ratio);
    update_param<double>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.bicycle_model.front_radius_ratio",
      vehicle_circles_bicycle_model_front_radius_ratio);

    // fitting uniform circles
    update_param<int>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.fitting_uniform_circle.num",
      vehicle_circles_fitting_uniform_circle_num);
  }

  {  // clearance
    update_param<double>(
      parameters, "mpt.clearance.hard_clearance_from_road", hard_clearance_from_road);
    update_param<double>(
      parameters, "mpt.clearance.soft_clearance_from_road", soft_clearance_from_road);
  }

  {  // weight
    update_param<double>(
      parameters, "mpt.weight.soft_collision_free_weight", soft_collision_free_weight);

    update_param<double>(parameters, "mpt.weight.lat_error_weight", lat_error_weight);
    update_param<double>(parameters, "mpt.weight.yaw_error_weight", yaw_error_weight);
    update_param<double>(parameters, "mpt.weight.yaw_error_rate_weight", yaw_error_rate_weight);
    update_param<double>(parameters, "mpt.weight.steer_input_weight", steer_input_weight);
    update_param<double>(parameters, "mpt.weight.steer_rate_weight", steer_rate_weight);

    update_param<double>(
      parameters, "mpt.weight.terminal_lat_error_weight", terminal_lat_error_weight);
    update_param<double>(
      parameters, "mpt.weight.terminal_yaw_error_weight", terminal_yaw_error_weight);
    update_param<double>(parameters, "mpt.weight.goal_lat_error_weight", goal_lat_error_weight);
    update_param<double>(parameters, "mpt.weight.goal_yaw_error_weight", goal_yaw_error_weight);
  }

  {  // avoidance
    update_param<double>(
      parameters, "mpt.avoidance.max_longitudinal_margin_for_bound_violation",
      max_longitudinal_margin_for_bound_violation);
    update_param<double>(parameters, "mpt.avoidance.max_bound_fixing_time", max_bound_fixing_time);
    update_param<double>(parameters, "mpt.avoidance.min_drivable_width", min_drivable_width);
    update_param<double>(parameters, "mpt.avoidance.max_avoidance_cost", max_avoidance_cost);
    update_param<double>(parameters, "mpt.avoidance.avoidance_cost_margin", avoidance_cost_margin);
    update_param<double>(
      parameters, "mpt.avoidance.avoidance_cost_band_length", avoidance_cost_band_length);
    update_param<double>(
      parameters, "mpt.avoidance.avoidance_cost_decrease_rate", avoidance_cost_decrease_rate);

    update_param<double>(
      parameters, "mpt.avoidance.weight.lat_error_weight", avoidance_lat_error_weight);
    update_param<double>(
      parameters, "mpt.avoidance.weight.yaw_error_weight", avoidance_yaw_error_weight);
    update_param<double>(
      parameters, "mpt.avoidance.weight.steer_input_weight", avoidance_steer_input_weight);
  }

  {  // validation
    update_param<double>(parameters, "mpt.validation.max_lat_error", max_validation_lat_error);
    update_param<double>(parameters, "mpt.validation.max_yaw_error", max_validation_yaw_error);
  }
}

MPTOptimizer::MPTOptimizer(
  rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const TrajectoryParam & traj_param, const std::shared_ptr<DebugData> debug_data_ptr,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
: enable_debug_info_(enable_debug_info),
  ego_nearest_param_(ego_nearest_param),
  vehicle_info_(vehicle_info),
  traj_param_(traj_param),
  debug_data_ptr_(debug_data_ptr),
  time_keeper_(time_keeper),
  logger_(node->get_logger().get_child("mpt_optimizer"))
{
  // initialize mpt param
  mpt_param_ = MPTParam(node, vehicle_info);
  updateVehicleCircles();
  debug_data_ptr_->mpt_visualize_sampling_num = mpt_param_.mpt_visualize_sampling_num;

  // state equation generator
  state_equation_generator_ =
    StateEquationGenerator(vehicle_info_.wheel_base_m, mpt_param_.max_steer_rad, time_keeper_);

  // osqp solver
  osqp_solver_ptr_ = std::make_unique<autoware::osqp_interface::OSQPInterface>(osqp_epsilon_);

  // publisher
  debug_fixed_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_fixed_traj", 1);
  debug_ref_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_ref_traj", 1);
  debug_mpt_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_traj", 1);
  debug_optimised_steering_pub_ =
    node->create_publisher<std_msgs::msg::Float32MultiArray>("~/debug/optimised_steering", 1);

  debug_acados_mpt_traj_pub_ = node->create_publisher<Trajectory>("~/debug/acados_mpt_traj", 1);
  debug_acados_optimised_steering_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
    "~/debug/acados_optimised_steering", 1);
}

void MPTOptimizer::updateVehicleCircles()
{
  const auto & p = mpt_param_;

  if (p.vehicle_circles_method == "uniform_circle") {
    std::tie(vehicle_circle_radiuses_, vehicle_circle_longitudinal_offsets_) =
      calcVehicleCirclesByUniformCircle(
        vehicle_info_, p.vehicle_circles_uniform_circle_num,
        p.vehicle_circles_uniform_circle_radius_ratio);
  } else if (p.vehicle_circles_method == "bicycle_model") {
    std::tie(vehicle_circle_radiuses_, vehicle_circle_longitudinal_offsets_) =
      calcVehicleCirclesByBicycleModel(
        vehicle_info_, p.vehicle_circles_bicycle_model_num,
        p.vehicle_circles_bicycle_model_rear_radius_ratio,
        p.vehicle_circles_bicycle_model_front_radius_ratio);
  } else if (p.vehicle_circles_method == "fitting_uniform_circle") {
    std::tie(vehicle_circle_radiuses_, vehicle_circle_longitudinal_offsets_) =
      calcVehicleCirclesByFittingUniformCircle(
        vehicle_info_, p.vehicle_circles_fitting_uniform_circle_num);
  } else {
    throw std::invalid_argument("mpt_param_.vehicle_circles_method is invalid.");
  }

  debug_data_ptr_->vehicle_circle_radiuses = vehicle_circle_radiuses_;
  debug_data_ptr_->vehicle_circle_longitudinal_offsets = vehicle_circle_longitudinal_offsets_;
}

void MPTOptimizer::initialize(const bool enable_debug_info, const TrajectoryParam & traj_param)
{
  enable_debug_info_ = enable_debug_info;
  traj_param_ = traj_param;
}

void MPTOptimizer::resetPreviousData()
{
  prev_ref_points_ptr_ = nullptr;
  prev_optimized_traj_points_ptr_ = nullptr;
}

void MPTOptimizer::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  mpt_param_.onParam(parameters);
  updateVehicleCircles();
  debug_data_ptr_->mpt_visualize_sampling_num = mpt_param_.mpt_visualize_sampling_num;
}
std::optional<std::vector<TrajectoryPoint>> MPTOptimizer::optimizeTrajectory(
  const PlannerData & planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & p = planner_data;
  const auto & traj_points = p.traj_points;

  // 1. calculate reference points
  auto [ref_points, ref_points_spline] = calcReferencePoints(planner_data, traj_points);
  if (ref_points.size() < 2) {
    RCLCPP_INFO_EXPRESSION(
      logger_, enable_debug_info_, "return std::nullopt since ref_points size is less than 2.");
    return std::nullopt;
  }

  std::optional<std::vector<TrajectoryPoint>> acados_traj_points;
  AcadosSolution acados_result;

  if (mpt_param_.use_acados) {
    acados_result = runAcadosMPT(ref_points_spline, p.ego_pose, vehicle_info_);

    std::cout << "utraj:" << std::endl;
    for (const auto & control : acados_result.utraj) {
      for (const auto val : control) {
        std::cout << val << " ";
      }
    }
    std::cout << std::endl;

    // Convert Acados solution to trajectory points
    acados_traj_points = convertAcadosSolutionToTrajectory(ref_points, acados_result);

    if (!acados_traj_points) {
      RCLCPP_WARN(logger_, "Failed to convert Acados solution to trajectory");
      return std::nullopt;
    }

    // Publish optimized steering (convert Acados controls to Eigen vector)
    Eigen::VectorXd steering_angles(acados_result.utraj.size());
    for (size_t i = 0; i < acados_result.utraj.size(); ++i) {
      steering_angles(i) = acados_result.utraj[i][0];  // Extract delta (steering)
    }

    debug_data_ptr_->ref_points = ref_points;
    prev_ref_points_ptr_ = std::make_shared<std::vector<ReferencePoint>>(ref_points);
    prev_optimized_traj_points_ptr_ =
      std::make_shared<std::vector<TrajectoryPoint>>(*acados_traj_points);

    std_msgs::msg::Float32MultiArray acados_steering_msg;
    for (const auto & delta : acados_result.utraj) {
      acados_steering_msg.data.push_back(static_cast<float>(delta[0]));
    }

    debug_acados_optimised_steering_pub_->publish(acados_steering_msg);

    // Publish acados trajectory to separate topic for comparison
    const auto acados_traj =
      autoware::motion_utils::convertToTrajectory(*acados_traj_points, p.header);

    debug_acados_mpt_traj_pub_->publish(acados_traj);
  }

  // 2. calculate B and W matrices where x = B u + W
  const auto mpt_mat = state_equation_generator_.calcMatrix(ref_points);

  // 3. calculate Q and R matrices where J(x, u) = x^t Q x + u^t R u
  const auto val_mat = calcValueMatrix(ref_points, traj_points);

  // 4. get objective matrix
  const auto obj_mat = calcObjectiveMatrix(mpt_mat, val_mat, ref_points);

  // 5. get constraints matrix
  const auto const_mat = calcConstraintMatrix(mpt_mat, ref_points);

  // 6. optimize steer angles
  const auto optimized_variables = calcOptimizedSteerAngles(ref_points, obj_mat, const_mat);
  if (!optimized_variables) {
    RCLCPP_WARN(logger_, "return std::nullopt since could not solve qp");

    return std::nullopt;
  }

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;

  publishOptimizedSteering(optimized_variables->segment(N_x, N_u));

  // 7. convert to points with validation
  auto mpt_traj_points = calcMPTPoints(ref_points, *optimized_variables, mpt_mat);
  if (!mpt_traj_points) {
    RCLCPP_WARN(logger_, "return std::nullopt since lateral or yaw error is too large.");
    return std::nullopt;
  }

  // 8. publish trajectories for debug
  publishDebugTrajectories(p.header, ref_points, *mpt_traj_points);

  debug_data_ptr_->ref_points = ref_points;
  prev_ref_points_ptr_ = std::make_shared<std::vector<ReferencePoint>>(ref_points);
  prev_optimized_traj_points_ptr_ =
    std::make_shared<std::vector<TrajectoryPoint>>(*mpt_traj_points);

  if (mpt_param_.use_acados) {
    return acados_traj_points;
  }

  return mpt_traj_points;
}

std::optional<std::vector<TrajectoryPoint>> MPTOptimizer::getPrevOptimizedTrajectoryPoints() const
{
  if (prev_optimized_traj_points_ptr_) {
    return *prev_optimized_traj_points_ptr_;
  }
  return std::nullopt;
}

void MPTOptimizer::publishOptimizedSteering(const Eigen::VectorXd & optimized_variables) const
{
  std_msgs::msg::Float32MultiArray msg;
  msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  msg.layout.dim[0].size = optimized_variables.size();
  msg.layout.dim[0].stride = optimized_variables.size();

  for (size_t i = 0; i < static_cast<size_t>(optimized_variables.size()); ++i) {
    msg.data.push_back(static_cast<float>(optimized_variables(i)));
  }

  debug_optimised_steering_pub_->publish(msg);
}

geometry_msgs::msg::Point getCorner(const geometry_msgs::msg::Pose & ego_pose, double dx, double dy)
{
  // Convert quaternion to roll, pitch, yaw
  tf2::Quaternion q(
    ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z, ego_pose.orientation.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  geometry_msgs::msg::Point p;
  p.x = ego_pose.position.x + dx * cos_yaw - dy * sin_yaw;
  p.y = ego_pose.position.y + dx * sin_yaw + dy * cos_yaw;
  p.z = ego_pose.position.z;
  return p;
}

// Compute cubic spline coefficients from knots and values (natural cubic spline)
// Returns flattened array of coefficients: [col0_row0, col0_row1, col0_row2, col0_row3, col1_row0,
// ...] matching scipy's CubicSpline with column-major ('F') flatten
std::vector<double> MPTOptimizer::computeCubicSplineCoeffs(
  const std::vector<double> & knots, const std::vector<double> & values) const
{
  const size_t n = knots.size();
  if (n < 2 || values.size() != n) {
    return std::vector<double>(4 * std::max(0ul, n - 1), 0.0);
  }

  const size_t n_segments = n - 1;
  std::vector<double> h(n_segments);
  for (size_t i = 0; i < n_segments; ++i) {
    h[i] = knots[i + 1] - knots[i];
    if (h[i] <= 0) {
      h[i] = 1e-6;  // Prevent division by zero
    }
  }

  std::vector<double> M(n, 0.0);
  if (n > 2) {
    std::vector<double> a(n), b(n), c(n), d(n);
    // Natural boundary: M[0] = 0
    b[0] = 1.0;
    c[0] = 0.0;
    d[0] = 0.0;

    for (size_t i = 1; i < n - 1; ++i) {
      a[i] = h[i - 1];
      b[i] = 2.0 * (h[i - 1] + h[i]);
      c[i] = h[i];
      d[i] = 6.0 * ((values[i + 1] - values[i]) / h[i] - (values[i] - values[i - 1]) / h[i - 1]);
    }

    // Natural boundary: M[n-1] = 0
    a[n - 1] = 0.0;
    b[n - 1] = 1.0;
    d[n - 1] = 0.0;

    // Thomas algorithm (tridiagonal solver)
    c[0] /= b[0];
    d[0] /= b[0];
    for (size_t i = 1; i < n; ++i) {
      double denom = b[i] - a[i] * c[i - 1];
      if (i < n - 1) c[i] /= denom;
      d[i] = (d[i] - a[i] * d[i - 1]) / denom;
    }
    M[n - 1] = d[n - 1];
    for (size_t i = n - 1; i > 0; --i) {
      M[i - 1] = d[i - 1] - c[i - 1] * M[i];
    }
  }

  // Compute cubic polynomial coefficients for each segment
  // SymbolicCubicSpline expects: [a_seg0, a_seg1, ..., b_seg0, b_seg1, ..., c_seg0, ..., d_seg0,
  // ...] where p(t) = a*t^3 + b*t^2 + c*t + d
  std::vector<double> a_coeffs, b_coeffs, c_coeffs, d_coeffs;
  a_coeffs.reserve(n_segments);
  b_coeffs.reserve(n_segments);
  c_coeffs.reserve(n_segments);
  d_coeffs.reserve(n_segments);

  for (size_t i = 0; i < n_segments; ++i) {
    double hi = h[i];
    double yi = values[i];
    double yi1 = values[i + 1];
    double Mi = M[i];
    double Mi1 = M[i + 1];

    double a = (Mi1 - Mi) / (6.0 * hi);
    double b = Mi / 2.0;
    double c = (yi1 - yi) / hi - hi * (2.0 * Mi + Mi1) / 6.0;
    double d = yi;

    a_coeffs.push_back(a);
    b_coeffs.push_back(b);
    c_coeffs.push_back(c);
    d_coeffs.push_back(d);
  }

  // Column-major flatten: all a's, then all b's, then all c's, then all d's
  std::vector<double> coeffs_flat;
  coeffs_flat.reserve(4 * n_segments);
  coeffs_flat.insert(coeffs_flat.end(), a_coeffs.begin(), a_coeffs.end());
  coeffs_flat.insert(coeffs_flat.end(), b_coeffs.begin(), b_coeffs.end());
  coeffs_flat.insert(coeffs_flat.end(), c_coeffs.begin(), c_coeffs.end());
  coeffs_flat.insert(coeffs_flat.end(), d_coeffs.begin(), d_coeffs.end());

  return coeffs_flat;
}

// Build parameter vector and initial state x0 from the request. If a parameter-size mismatch
// is detected, this will set skipSolve=true and populate resp with empty results.
std::array<double, NP> MPTOptimizer::buildParameters(
  const double e_y_ego, const double e_psi_ego, const std::vector<double> & knots_in,
  const std::vector<double> & x_coeffs_flat_in, const std::vector<double> & y_coeffs_flat_in,
  const std::vector<double> & curvatures_in,
  const std::vector<geometry_msgs::msg::Point> & body_points,
  const std::vector<geometry_msgs::msg::Point> & body_points_curvilinear,
  std::array<double, NX> & x0) const
{
  // Make copies so we can modify them
  std::vector<double> knots = knots_in;
  std::vector<double> x_coeffs_flat = x_coeffs_flat_in;
  std::vector<double> y_coeffs_flat = y_coeffs_flat_in;
  std::vector<double> curvatures = curvatures_in;

  size_t n_segments = (int)knots.size() - 1;
  RCLCPP_ERROR(logger_, "Received request with %ld segments", n_segments);

  std::cerr << "Initial sizes: knots=" << knots.size() << " x_coeffs=" << x_coeffs_flat.size()
            << " y_coeffs=" << y_coeffs_flat.size() << " curvatures=" << curvatures.size()
            << std::endl;

  size_t target_n_knots = CURVILINEAR_BICYCLE_MODEL_SPATIAL_N;
  size_t target_segments = target_n_knots - 1;

  // Adjust sizes if necessary (simple strategy: extend last values)
  if (n_segments < target_segments) {
    RCLCPP_ERROR(
      logger_, "Extending from %ld to %ld segments (to match %ld knots)", n_segments,
      target_segments, target_n_knots);
    size_t n_missing = target_segments - n_segments;
    double last_knot = knots.back();
    double ds = mpt_param_.delta_arc_length;

    for (size_t i = 0; i < n_missing; ++i) {
      knots.push_back(last_knot + (i + 1) * ds);
    }

    // extend coeffs: linear extension (straight line)
    // Coefficients are stored column-major: [a0, a1, ..., an, b0, b1, ..., bn, c0, c1, ..., cn, d0,
    // d1, ..., dn] For linear extension: a=0, b=0, c=derivative at end, d=value at end
    size_t last_seg_idx = n_segments - 1;
    double last_a_x = x_coeffs_flat[last_seg_idx * 4 + 0];
    double last_b_x = x_coeffs_flat[last_seg_idx * 4 + 1];
    double last_c_x = x_coeffs_flat[last_seg_idx * 4 + 2];
    double last_d_x = x_coeffs_flat[last_seg_idx * 4 + 3];

    double last_a_y = y_coeffs_flat[last_seg_idx * 4 + 0];
    double last_b_y = y_coeffs_flat[last_seg_idx * 4 + 1];
    double last_c_y = y_coeffs_flat[last_seg_idx * 4 + 2];
    double last_d_y = y_coeffs_flat[last_seg_idx * 4 + 3];

    // Evaluate last segment at s=1 (end of segment) to get value and derivative
    // f(s) = a*s^3 + b*s^2 + c*s + d
    // f'(s) = 3*a*s^2 + 2*b*s + c
    // At s=1: value = a + b + c + d, derivative = 3*a + 2*b + c
    double end_val_x = last_a_x + last_b_x + last_c_x + last_d_x;
    double end_deriv_x = 3.0 * last_a_x + 2.0 * last_b_x + last_c_x;

    double end_val_y = last_a_y + last_b_y + last_c_y + last_d_y;
    double end_deriv_y = 3.0 * last_a_y + 2.0 * last_b_y + last_c_y;

    // For linear extension: a=0, b=0, c=derivative, d=value
    // Add all a coefficients (zeros for linear)
    for (size_t j = 0; j < n_missing; ++j) {
      x_coeffs_flat.push_back(0.0);
      y_coeffs_flat.push_back(0.0);
    }

    // Add all b coefficients (zeros for linear)
    for (size_t j = 0; j < n_missing; ++j) {
      x_coeffs_flat.push_back(0.0);
      y_coeffs_flat.push_back(0.0);
    }

    // Add all c coefficients (derivative for linear continuation)
    for (size_t j = 0; j < n_missing; ++j) {
      x_coeffs_flat.push_back(end_deriv_x);
      y_coeffs_flat.push_back(end_deriv_y);
    }

    // Add all d coefficients (starting value for linear continuation)
    for (size_t j = 0; j < n_missing; ++j) {
      x_coeffs_flat.push_back(end_val_x);
      y_coeffs_flat.push_back(end_val_y);
    }

    // Add curvatures (zero for linear extension)
    for (size_t j = 0; j < n_missing; ++j) {
      curvatures.push_back(0.0);
    }

    std::cerr << "Extended knots to: " << knots.size() << std::endl;
    std::cerr << "Extended x_coeffs to: " << x_coeffs_flat.size() << std::endl;
    std::cerr << "Extended y_coeffs to: " << y_coeffs_flat.size() << std::endl;
    std::cerr << "Extended curvatures to: " << curvatures.size() << std::endl;

  } else if (n_segments > target_segments) {
    RCLCPP_ERROR(
      logger_, "Clipping from %ld to %ld segments (to match %ld knots)", n_segments,
      target_segments, target_n_knots);
    // clip to exactly target_n_knots knots (which gives target_segments segments)
    knots.resize(target_n_knots);
    x_coeffs_flat.resize(4 * target_segments);
    y_coeffs_flat.resize(4 * target_segments);
    curvatures.resize(target_n_knots);
  }

  // Ensure curvatures has same size as knots for cubic spline interpolation
  if (curvatures.size() < knots.size()) {
    double last_k = 0.0;
    while (curvatures.size() < knots.size()) {
      curvatures.push_back(last_k);
    }
  } else if (curvatures.size() > knots.size()) {
    curvatures.resize(knots.size());
  }

  RCLCPP_ERROR(
    logger_, "sizes: knots=%zu x_coeffs=%zu y_coeffs=%zu curvatures=%zu body_points=%zu",
    knots.size(), x_coeffs_flat.size(), y_coeffs_flat.size(), curvatures.size(),
    body_points.size());

  if (body_points.size() != body_points_curvilinear.size()) {
    RCLCPP_ERROR(
      logger_, "body points length mismatch: body_points=%zu body_points_curvilinear=%zu",
      body_points.size(), body_points_curvilinear.size());
    assert(body_points.size() == body_points_curvilinear.size() && "body points mismatch");
  }

  // body points curvilinear -> vector of doubles (s values then eY values)
  std::vector<double> body_points_curvilinear_vec;
  for (const auto & pt : body_points_curvilinear) body_points_curvilinear_vec.push_back(pt.x);
  for (const auto & pt : body_points_curvilinear) body_points_curvilinear_vec.push_back(pt.y);

  // body points global
  std::vector<double> body_points_xy;
  for (const auto & pt : body_points) body_points_xy.push_back(pt.x);
  for (const auto & pt : body_points) body_points_xy.push_back(pt.y);

  // Build parameters vector similar to Python
  std::array<double, NP> parameters;
  double s_interp = 0.0;
  size_t idx = 0;

  // 1. s_interp
  parameters[idx++] = s_interp;

  // 2. knots
  for (double v : knots) {
    parameters[idx++] = v;
  }

  // 3. x_coeffs_flat
  for (double v : x_coeffs_flat) {
    parameters[idx++] = v;
  }

  // 4. knots again
  for (double v : knots) {
    parameters[idx++] = v;
  }

  // 5. y_coeffs_flat
  for (double v : y_coeffs_flat) {
    parameters[idx++] = v;
  }

  // 6. knots again
  for (double v : knots) {
    parameters[idx++] = v;
  }

  // 7. Compute cubic spline coefficients from curvatures
  // Python uses: ClothoidSpline -> CubicSpline(knots, curvatures) -> coeffs (4×(N-1))
  // We need to fit a cubic spline to (knots, curvatures) and extract the 4×(N-1) coefficients

  const std::vector<double> clothoid_coeffs_flat = computeCubicSplineCoeffs(knots, curvatures);

  for (double v : clothoid_coeffs_flat) {
    parameters[idx++] = v;
  }

  std::cerr << vehicle_info_.vehicle_length_m << ", " << vehicle_info_.vehicle_width_m << ", "
            << vehicle_info_.min_longitudinal_offset_m << ", "
            << vehicle_info_.max_longitudinal_offset_m << std::endl;

  const double lf = vehicle_info_.max_longitudinal_offset_m;
  const double lr = vehicle_info_.min_longitudinal_offset_m;

  parameters[idx++] = lf;
  parameters[idx++] = lr;

  // set x0: initial state vector
  idx = 0;
  x0[idx++] = e_y_ego;
  x0[idx++] = e_psi_ego;
  for (const auto & body_point_curvilinear : body_points_curvilinear_vec) {
    x0[idx++] = body_point_curvilinear;
  }

  return parameters;
}

void MPTOptimizer::setParametersToSolver(const std::array<double, NP> & parameters)
{
  for (size_t stage = 0; stage < (int)N; ++stage) {
    std::array<double, NP> params_copy = parameters;
    double s_interp = 0.0;
    const double sref = mpt_param_.num_points * mpt_param_.delta_arc_length;
    if (sref > 0.0) {
      s_interp = sref * ((double)stage / (double)N);
    }
    params_copy[0] = s_interp;
    acados_interface_.setParameters(stage, params_copy);
  }
}

std::optional<std::vector<TrajectoryPoint>> MPTOptimizer::convertAcadosSolutionToTrajectory(
  std::vector<ReferencePoint> & ref_points, const AcadosSolution & acados_solution) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // Check if solution is valid
  if (acados_solution.status > 2) {
    RCLCPP_WARN(logger_, "Acados solver returned non-zero status: %d", acados_solution.status);
    return std::nullopt;
  }

  // The Acados state vector contains:
  // x[0] = eY (lateral error)
  // x[1] = eψ (yaw/heading error)
  // x[2:2+num_body_points] = s_body_points (not used for main trajectory)
  // x[2+num_body_points:] = eY_body_points (not used for main trajectory)

  // The control vector contains:
  // u[0] = delta (steering angle)

  const size_t N_ref = ref_points.size();

  std::vector<TrajectoryPoint> traj_points;
  traj_points.reserve(ref_points.size());

  for (size_t i = 0; i < N_ref; ++i) {
    auto & ref_point = ref_points.at(i);
    const auto & state = acados_solution.xtraj[i];

    // Extract lateral and yaw errors from Acados state
    const double lat_error = state[0];  // eY
    const double yaw_error = state[1];  // eψ

    // Validate optimization result
    if (mpt_param_.enable_optimization_validation) {
      if (
        mpt_param_.max_validation_lat_error < std::abs(lat_error) ||
        mpt_param_.max_validation_yaw_error < std::abs(yaw_error)) {
        RCLCPP_WARN(
          logger_, "Acados solution validation failed at index %zu: lat_error=%.3f, yaw_error=%.3f",
          i, lat_error, yaw_error);
        return std::nullopt;
      }
    }

    // Store optimization result
    ref_point.optimized_kinematic_state = KinematicState{lat_error, yaw_error};

    // Store steering input (last state has no control)
    if (i == N_ref - 1) {
      ref_point.optimized_input = 0.0;
    } else {
      ref_point.optimized_input = acados_solution.utraj[i][0];
    }

    // Create trajectory point with updated pose and velocity
    TrajectoryPoint traj_point;
    traj_point.pose = ref_point.offsetDeviation(lat_error, yaw_error);
    traj_point.longitudinal_velocity_mps = ref_point.longitudinal_velocity_mps;

    traj_points.push_back(traj_point);
  }

  return traj_points;
}

AcadosSolution MPTOptimizer::runAcadosMPT(
  const autoware::interpolation::SplineInterpolationPoints2d & ref_points_spline,
  const geometry_msgs::msg::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  // Get spline coefficients for x and y
  const auto & knots = ref_points_spline.getSplineKnots();
  const auto & x_coeffs = ref_points_spline.getSplineCoefficientsX();
  const auto & y_coeffs = ref_points_spline.getSplineCoefficientsY();
  const auto & curvatures = ref_points_spline.getSplineInterpolatedCurvatures();

  const double half_width =
    (vehicle_info.vehicle_width_m + vehicle_info.left_overhang_m + vehicle_info.right_overhang_m) /
    2.0;

  // max_longitudinal_offset_m already includes front_overhang_m + wheel_base_m
  // min_longitudinal_offset_m is already -rear_overhang_m
  // So front = max_longitudinal_offset_m (distance from center to front)
  // And rear = -min_longitudinal_offset_m (distance from center to rear, positive value)
  const double front = vehicle_info.max_longitudinal_offset_m;
  const double rear = vehicle_info.min_longitudinal_offset_m;

  // const auto [s_ego, e_y_ego] =
  //   ref_points_spline.projectPointOntoSpline(ego_pose.position.x, ego_pose.position.y);

  // const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  // const double ref_yaw = ref_points_spline.getSplineInterpolatedYaw(0, s_ego);
  // const double e_psi_ego = ego_yaw - ref_yaw;

  const size_t num_body_points = 6;

  // Use actual ego pose (not projected) to transform body points to global frame
  const std::array<geometry_msgs::msg::Point, num_body_points> boundary_points_global_frame = {
    getCorner(ego_pose, front, half_width),   // front-left
    getCorner(ego_pose, front, -half_width),  // front-right
    getCorner(ego_pose, front + 0.50 * (rear - front), -half_width),
    getCorner(ego_pose, -rear, -half_width),  // rear-right
    getCorner(ego_pose, -rear, half_width),   // rear-left
    getCorner(ego_pose, rear + 0.50 * (front - rear), half_width),
  };

  std::array<geometry_msgs::msg::Point, num_body_points> corner_points_curvilinear;
  std::transform(
    boundary_points_global_frame.begin(), boundary_points_global_frame.end(),
    corner_points_curvilinear.begin(), [&ref_points_spline](const geometry_msgs::msg::Point & p) {
      const auto [s, e_y] = ref_points_spline.projectPointOntoSpline(p.x, p.y);
      geometry_msgs::msg::Point projected;
      projected.x = s;
      projected.y = e_y;  // Don't clip - let optimizer see true errors
      projected.z = 0.0;
      return projected;
    });

  const auto [s_ego, e_y_ego] =
    ref_points_spline.projectPointOntoSpline(ego_pose.position.x, ego_pose.position.y);
  std::cout << "s_ego: " << s_ego << ", e_y_ego: " << e_y_ego << std::endl;

  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  const double ref_yaw = ref_points_spline.getSplineInterpolatedYaw(0, s_ego);
  const double e_psi_ego = ego_yaw - ref_yaw;

  // Convert arrays to vectors for buildParameters
  std::vector<double> knots_vec(knots.begin(), knots.end());
  std::vector<double> x_coeffs_vec(x_coeffs.begin(), x_coeffs.end());
  std::vector<double> y_coeffs_vec(y_coeffs.begin(), y_coeffs.end());
  std::vector<double> curvatures_vec(curvatures.begin(), curvatures.end());

  std::vector<geometry_msgs::msg::Point> body_points_vec(
    boundary_points_global_frame.begin(), boundary_points_global_frame.end());
  std::vector<geometry_msgs::msg::Point> body_points_curvilinear_vec(
    corner_points_curvilinear.begin(), corner_points_curvilinear.end());

  std::array<double, NX> x0;
  // x0[0] = e_y_ego;
  // x0[1] = e_psi_ego;

  std::array<double, NP> parameters = buildParameters(
    e_y_ego, e_psi_ego, knots_vec, x_coeffs_vec, y_coeffs_vec, curvatures_vec, body_points_vec,
    body_points_curvilinear_vec, x0);

  setParametersToSolver(parameters);

  AcadosSolution acados_solution = acados_interface_.getControl(x0);

  return acados_solution;
}

std::pair<std::vector<ReferencePoint>, autoware::interpolation::SplineInterpolationPoints2d>
MPTOptimizer::calcReferencePoints(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & p = planner_data;

  const double forward_traj_length = mpt_param_.num_points * mpt_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;

  // 1. resample and convert smoothed points type from trajectory points to reference points
  time_keeper_->start_track("resampleReferencePoints");
  std::vector<ReferencePoint> ref_points = [&]() {
    const auto resampled_smoothed_points =
      trajectory_utils::resampleTrajectoryPointsWithoutStopPoint(
        smoothed_points, mpt_param_.delta_arc_length);
    return trajectory_utils::convertToReferencePoints(resampled_smoothed_points);
  }();
  time_keeper_->end_track("resampleReferencePoints");

  // 2. crop forward and backward with margin, and calculate spline interpolation
  // NOTE: Margin is added to calculate orientation, curvature, etc precisely.
  //       Start point may change. Spline calculation is required.
  constexpr double tmp_margin = 10.0;
  size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);
  ref_points = autoware::motion_utils::cropPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length + tmp_margin,
    backward_traj_length + tmp_margin);

  // remove repeated points
  ref_points = trajectory_utils::sanitizePoints(ref_points);
  autoware::interpolation::SplineInterpolationPoints2d ref_points_spline(ref_points);
  ego_seg_idx = trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);

  // 3. calculate orientation and curvature
  updateOrientation(ref_points, ref_points_spline);
  updateCurvature(ref_points, ref_points_spline);

  // 4. crop backward
  // NOTE: Start point may change. Spline calculation is required.
  ref_points = autoware::motion_utils::cropPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length + tmp_margin,
    backward_traj_length);
  ref_points_spline = autoware::interpolation::SplineInterpolationPoints2d(ref_points);
  ego_seg_idx = trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);

  // 5. update fixed points, and resample
  // NOTE: This must be after backward cropping.
  //       New start point may be added and resampled. Spline calculation is required.
  updateFixedPoint(ref_points);
  ref_points = trajectory_utils::sanitizePoints(ref_points);
  ref_points_spline = autoware::interpolation::SplineInterpolationPoints2d(ref_points);

  // 6. update bounds
  // NOTE: After this, resample must not be called since bounds are not interpolated.
  updateBounds(ref_points, p.left_bound, p.right_bound, p.ego_pose, p.ego_vel);
  updateVehicleBounds(ref_points, ref_points_spline);

  // 7. update delta arc length
  updateDeltaArcLength(ref_points);

  // 8. update extra information (alpha and beta)
  // NOTE: This must be after calculation of bounds and delta arc length
  updateExtraPoints(ref_points);

  // 9. crop forward
  ref_points = autoware::motion_utils::cropForwardPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length);

  if (ref_points_spline.getSize() == 0) {
    return std::make_pair(ref_points, ref_points_spline);
  }

  return std::make_pair(ref_points, ref_points_spline);
}

void MPTOptimizer::updateOrientation(
  std::vector<ReferencePoint> & ref_points,
  const autoware::interpolation::SplineInterpolationPoints2d & ref_points_spline) const
{
  const auto yaw_vec = ref_points_spline.getSplineInterpolatedYaws();
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ref_points.at(i).pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw_vec.at(i));
  }
}

void MPTOptimizer::updateCurvature(
  std::vector<ReferencePoint> & ref_points,
  const autoware::interpolation::SplineInterpolationPoints2d & ref_points_spline) const
{
  const auto curvature_vec = ref_points_spline.getSplineInterpolatedCurvatures();
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ref_points.at(i).curvature = curvature_vec.at(i);
  }
}

void MPTOptimizer::updateFixedPoint(std::vector<ReferencePoint> & ref_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!prev_ref_points_ptr_) {
    // no fixed point
    return;
  }

  // replace the front pose and curvature with previous reference points
  const auto idx = trajectory_utils::updateFrontPointForFix(
    ref_points, *prev_ref_points_ptr_, mpt_param_.delta_arc_length, ego_nearest_param_);

  // NOTE: memorize front point to be fixed before resampling
  const auto front_point = ref_points.front();

  if (idx && *idx != 0) {
    // In order to fix the front "orientation" defined by two front points, insert the previous
    // fixed point.
    ref_points.insert(ref_points.begin(), prev_ref_points_ptr_->at(static_cast<int>(*idx) - 1));

    // resample to make ref_points' interval constant.
    // NOTE: Only pose, velocity and curvature will be interpolated.
    ref_points = trajectory_utils::resampleReferencePoints(ref_points, mpt_param_.delta_arc_length);

    // update pose which is previous one, and fixed kinematic state
    // NOTE: There may be a lateral error between the previous and input points.
    //       Therefore, the pose for fix should not be resampled.
    const auto & prev_ref_front_point = prev_ref_points_ptr_->at(*idx);
    const auto & prev_ref_prev_front_point = prev_ref_points_ptr_->at(static_cast<int>(*idx) - 1);

    ref_points.front().pose = prev_ref_prev_front_point.pose;
    ref_points.front().fixed_kinematic_state = prev_ref_prev_front_point.optimized_kinematic_state;
    ref_points.at(1).pose = prev_ref_front_point.pose;
    ref_points.at(1).fixed_kinematic_state = prev_ref_front_point.optimized_kinematic_state;
  } else {
    // resample to make ref_points' interval constant.
    // NOTE: Only pose, velocity and curvature will be interpolated.
    ref_points = trajectory_utils::resampleReferencePoints(ref_points, mpt_param_.delta_arc_length);

    ref_points.front().pose = front_point.pose;
    ref_points.front().curvature = front_point.curvature;
    ref_points.front().fixed_kinematic_state = front_point.optimized_kinematic_state;
  }
}

void MPTOptimizer::updateDeltaArcLength(std::vector<ReferencePoint> & ref_points) const
{
  for (size_t i = 0; i < ref_points.size(); i++) {
    ref_points.at(i).delta_arc_length =
      (i == ref_points.size() - 1)
        ? 0.0
        : autoware_utils::calc_distance2d(ref_points.at(i + 1), ref_points.at(i));
  }
}

void MPTOptimizer::updateExtraPoints(std::vector<ReferencePoint> & ref_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // alpha
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto front_wheel_pos =
      trajectory_utils::getNearestPosition(ref_points, i, vehicle_info_.wheel_base_m);

    const bool are_too_close_points =
      autoware_utils::calc_distance2d(front_wheel_pos, ref_points.at(i).pose.position) < 1e-03;
    const auto front_wheel_yaw =
      are_too_close_points
        ? ref_points.at(i).getYaw()
        : autoware_utils::calc_azimuth_angle(ref_points.at(i).pose.position, front_wheel_pos);
    ref_points.at(i).alpha =
      autoware_utils::normalize_radian(front_wheel_yaw - ref_points.at(i).getYaw());
  }

  {  // avoidance
    // calculate one-step avoidance const
    for (size_t i = 0; i < ref_points.size(); ++i) {
      const auto normalized_avoidance_cost = calcNormalizedAvoidanceCost(ref_points.at(i));
      if (normalized_avoidance_cost) {
        const int max_length_idx =
          std::floor(mpt_param_.avoidance_cost_band_length / mpt_param_.delta_arc_length);
        for (int j = -max_length_idx; j <= max_length_idx; ++j) {
          const int k = i + j;
          if (0 <= k && k < static_cast<int>(ref_points.size())) {
            ref_points.at(k).normalized_avoidance_cost = *normalized_avoidance_cost;
          }
        }
      }
    }

    /*
    // update avoidance cost between longitudinally close obstacles
    constexpr double max_longitudinal_length_to_fill_drivable_area = 50;
    const int edge_fill_index = std::ceil(max_longitudinal_length_to_fill_drivable_area /
    mpt_param_.delta_arc_length / 2); const auto copied_ref_points = ref_points; for (size_t i = 0;
    i < ref_points.size(); ++i) { const double base_normalized_avoidance_cost =
    ref_points.at(i).normalized_avoidance_cost; for (int j = -edge_fill_index; j <= edge_fill_index;
    ++j) { const int k = i + j; if (k < 0 || ref_points.size() - 1 <= k) { continue;
        }
        ref_points.at(i).normalized_avoidance_cost =
    std::max(ref_points.at(i).normalized_avoidance_cost,
    copied_ref_points.at(k).normalized_avoidance_cost);
      }
    }
    */

    // update spread avoidance cost
    for (int i = 0; i < static_cast<int>(ref_points.size()); ++i) {
      const double base_normalized_avoidance_cost = ref_points.at(i).normalized_avoidance_cost;
      if (0 < base_normalized_avoidance_cost) {
        const int edge_decrease_idx = std::floor(
          ref_points.at(i).normalized_avoidance_cost / mpt_param_.avoidance_cost_decrease_rate);
        for (int j = -edge_decrease_idx; j <= edge_decrease_idx; ++j) {
          const int k = i + j;
          if (0 <= k && k < static_cast<int>(ref_points.size())) {
            const double normalized_avoidance_cost = std::max(
              base_normalized_avoidance_cost -
                std::abs(j) * mpt_param_.avoidance_cost_decrease_rate,
              ref_points.at(k).normalized_avoidance_cost);
            ref_points.at(k).normalized_avoidance_cost =
              std::clamp(normalized_avoidance_cost, 0.0, 1.0);
          }
        }
      }
    }

    // take over previous avoidance cost
    const double max_dist_threshold = mpt_param_.delta_arc_length / 2.0;
    if (prev_ref_points_ptr_ && !prev_ref_points_ptr_->empty()) {
      for (int i = 0; i < static_cast<int>(ref_points.size()); ++i) {
        const size_t prev_idx = trajectory_utils::findEgoIndex(
          *prev_ref_points_ptr_, autoware_utils::get_pose(ref_points.at(i)), ego_nearest_param_);

        const double dist_to_prev =
          autoware_utils::calc_distance2d(ref_points.at(i), prev_ref_points_ptr_->at(prev_idx));
        if (max_dist_threshold < dist_to_prev) {
          continue;
        }

        ref_points.at(i).normalized_avoidance_cost = std::max(
          prev_ref_points_ptr_->at(prev_idx).normalized_avoidance_cost,
          ref_points.at(i).normalized_avoidance_cost);
      }
    }
  }
}

void MPTOptimizer::updateBounds(
  std::vector<ReferencePoint> & ref_points,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const geometry_msgs::msg::Pose & ego_pose, const double ego_vel) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const double soft_road_clearance =
    mpt_param_.soft_clearance_from_road + vehicle_info_.vehicle_width_m / 2.0;

  // calculate distance to left/right bound on each reference point
  // NOTE: Reference points is sometimes not fully covered by the drivable area.
  //       In some edge cases like U-turn, the wrong bound may be found. To avoid finding the wrong
  //       bound, some beginning bounds are copied from the following one.
  const size_t min_ref_point_index = std::min(
    static_cast<size_t>(std::ceil(1.0 / mpt_param_.delta_arc_length)), ref_points.size() - 1);
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto ref_point_for_bound_search = ref_points.at(std::max(min_ref_point_index, i));
    const double dist_to_left_bound = calcLateralDistToBounds(
      ref_point_for_bound_search.pose, left_bound, soft_road_clearance, true);
    const double dist_to_right_bound = calcLateralDistToBounds(
      ref_point_for_bound_search.pose, right_bound, soft_road_clearance, false);
    ref_points.at(i).bounds = Bounds{dist_to_right_bound, dist_to_left_bound};
  }

  // keep vehicle width + margin
  // NOTE: The drivable area's width is sometimes narrower than the vehicle width which means
  // infeasible to run especially when obstacles are extracted from the drivable area.
  //       In this case, the drivable area's width is forced to be wider.
  keepMinimumBoundsWidth(ref_points);

  // extend violated bounds, where the input path is outside the drivable area
  ref_points = extendViolatedBounds(ref_points);

  // keep previous boundary's width around ego to avoid sudden steering
  avoidSuddenSteering(ref_points, ego_pose, ego_vel);

  /*
  // TODO(murooka) deal with filling data between obstacles
  // fill between obstacles
  constexpr double max_longitudinal_length_to_fill_drivable_area = 20;
  const int edge_fill_index = std::ceil(max_longitudinal_length_to_fill_drivable_area /
  mpt_param_.delta_arc_length / 2); for (int i = 0; i < ref_points.size(); ++i) { for (int j =
  -edge_fill_index; j <= edge_fill_index; ++j) { const int k = i + j; if (k < 0 || ref_points.size()
  - 1 <= k) { continue;
      }

      const auto normalized_avoidance_cost = calcNormalizedAvoidanceCost(ref_points.at(k));
      if (normalized_avoidance_cost) {
      }
    }
  }
  */
  return;
}

void MPTOptimizer::keepMinimumBoundsWidth(std::vector<ReferencePoint> & ref_points) const
{
  // calculate drivable area width considering the curvature
  std::vector<double> min_dynamic_drivable_width_vec;
  for (int i = 0; i < static_cast<int>(ref_points.size()); ++i) {
    double curvature = std::abs(ref_points.at(i).curvature);
    if (i != static_cast<int>(ref_points.size()) - 1) {
      curvature = std::max(curvature, std::abs(ref_points.at(i + 1).curvature));
    }
    if (i != 0) {
      curvature = std::max(curvature, std::abs(ref_points.at(i - 1).curvature));
    }

    const double max_longitudinal_length = std::max(
      std::abs(vehicle_info_.max_longitudinal_offset_m),
      std::abs(vehicle_info_.min_longitudinal_offset_m));
    const double turning_radius = 1.0 / curvature;
    const double additional_drivable_width_by_curvature =
      std::hypot(max_longitudinal_length, turning_radius + vehicle_info_.vehicle_width_m / 2.0) -
      turning_radius - vehicle_info_.vehicle_width_m / 2.0;
    min_dynamic_drivable_width_vec.push_back(
      mpt_param_.min_drivable_width + additional_drivable_width_by_curvature);
  }

  // 1. calculate start and end sections which are out of bounds
  std::vector<std::pair<size_t, size_t>> out_of_upper_bound_sections;
  std::vector<std::pair<size_t, size_t>> out_of_lower_bound_sections;
  std::optional<size_t> out_of_upper_bound_start_idx = std::nullopt;
  std::optional<size_t> out_of_lower_bound_start_idx = std::nullopt;
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto & b = ref_points.at(i).bounds;

    // const double drivable_width = b.upper_bound - b.lower_bound;
    // const bool is_infeasible_to_drive = drivable_width < min_dynamic_drivable_width

    // NOTE: The following condition should be uncommented to see obstacles outside the path.
    //       However, on a narrow road, the ego may go outside the road border with this condition.
    //       Currently, we cannot distinguish obstacles and road border
    if (/*is_infeasible_to_drive ||*/ b.upper_bound < 0.0) {  // out of upper bound
      if (!out_of_upper_bound_start_idx) {
        out_of_upper_bound_start_idx = i;
      }
    } else {
      if (out_of_upper_bound_start_idx) {
        out_of_upper_bound_sections.push_back({*out_of_upper_bound_start_idx, i - 1});
        out_of_upper_bound_start_idx = std::nullopt;
      }
    }
    if (/*is_infeasible_to_drive ||*/ 0.0 < b.lower_bound) {  // out of lower bound
      if (!out_of_lower_bound_start_idx) {
        out_of_lower_bound_start_idx = i;
      }
    } else {
      if (out_of_lower_bound_start_idx) {
        out_of_lower_bound_sections.push_back({*out_of_lower_bound_start_idx, i - 1});
        out_of_lower_bound_start_idx = std::nullopt;
      }
    }
  }
  if (out_of_upper_bound_start_idx) {
    out_of_upper_bound_sections.push_back({*out_of_upper_bound_start_idx, ref_points.size() - 1});
  }
  if (out_of_lower_bound_start_idx) {
    out_of_lower_bound_sections.push_back({*out_of_lower_bound_start_idx, ref_points.size() - 1});
  }

  auto original_ref_points = ref_points;
  const auto is_inside_sections = [&](const size_t target_idx, const auto & sections) {
    for (const auto & section : sections) {
      if (section.first <= target_idx && target_idx <= section.second) {
        return true;
      }
    }
    return false;
  };

  // lower bound
  for (const auto & out_of_lower_bound_section : out_of_lower_bound_sections) {
    std::optional<size_t> upper_bound_start_idx = std::nullopt;
    std::optional<size_t> upper_bound_end_idx = std::nullopt;
    for (size_t p_idx = out_of_lower_bound_section.first;
         p_idx <= out_of_lower_bound_section.second; ++p_idx) {
      const bool is_out_of_upper_bound = is_inside_sections(p_idx, out_of_upper_bound_sections);

      const auto & original_b = original_ref_points.at(p_idx).bounds;
      auto & b = ref_points.at(p_idx).bounds;
      if (is_out_of_upper_bound) {
        if (!upper_bound_start_idx) {
          upper_bound_start_idx = p_idx;
        }
        upper_bound_end_idx = p_idx;

        // It seems both bounds are cut out. Widen the bounds towards the both side.
        const double center_dist_to_bounds =
          (original_b.upper_bound + original_b.lower_bound) / 2.0;
        b.upper_bound = std::max(
          b.upper_bound, center_dist_to_bounds + min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        b.lower_bound = std::min(
          b.lower_bound, center_dist_to_bounds - min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        continue;
      }
      // Only the Lower bound is cut out. Widen the bounds towards the lower bound since cut out too
      // much.
      b.lower_bound =
        std::min(b.lower_bound, original_b.upper_bound - min_dynamic_drivable_width_vec.at(p_idx));
    }
    // extend longitudinal if it overlaps out_of_upper_bound_sections
    if (upper_bound_start_idx) {
      for (size_t p_idx = out_of_lower_bound_section.first; p_idx < *upper_bound_start_idx;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.lower_bound =
          std::min(b.lower_bound, ref_points.at(*upper_bound_start_idx).bounds.lower_bound);
      }
    }
    if (upper_bound_end_idx) {
      for (size_t p_idx = *upper_bound_end_idx + 1; p_idx <= out_of_lower_bound_section.second;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.lower_bound =
          std::min(b.lower_bound, ref_points.at(*upper_bound_end_idx).bounds.lower_bound);
      }
    }
  }

  // upper bound
  for (const auto & out_of_upper_bound_section : out_of_upper_bound_sections) {
    std::optional<size_t> lower_bound_start_idx = std::nullopt;
    std::optional<size_t> lower_bound_end_idx = std::nullopt;
    for (size_t p_idx = out_of_upper_bound_section.first;
         p_idx <= out_of_upper_bound_section.second; ++p_idx) {
      const bool is_out_of_lower_bound = is_inside_sections(p_idx, out_of_lower_bound_sections);

      const auto & original_b = original_ref_points.at(p_idx).bounds;
      auto & b = ref_points.at(p_idx).bounds;
      if (is_out_of_lower_bound) {
        if (!lower_bound_start_idx) {
          lower_bound_start_idx = p_idx;
        }
        lower_bound_end_idx = p_idx;

        // It seems both bounds are cut out. Widen the bounds towards the both side.
        const double center_dist_to_bounds =
          (original_b.upper_bound + original_b.lower_bound) / 2.0;
        b.upper_bound = std::max(
          b.upper_bound, center_dist_to_bounds + min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        b.lower_bound = std::min(
          b.lower_bound, center_dist_to_bounds - min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        continue;
      }
      // Only the Upper bound is cut out. Widen the bounds towards the upper bound since cut out too
      // much.
      b.upper_bound =
        std::max(b.upper_bound, original_b.lower_bound + min_dynamic_drivable_width_vec.at(p_idx));
    }
    // extend longitudinal if it overlaps out_of_lower_bound_sections
    if (lower_bound_start_idx) {
      for (size_t p_idx = out_of_upper_bound_section.first; p_idx < *lower_bound_start_idx;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.upper_bound =
          std::max(b.upper_bound, ref_points.at(*lower_bound_start_idx).bounds.upper_bound);
      }
    }
    if (lower_bound_end_idx) {
      for (size_t p_idx = *lower_bound_end_idx + 1; p_idx <= out_of_upper_bound_section.second;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.upper_bound =
          std::max(b.upper_bound, ref_points.at(*lower_bound_end_idx).bounds.upper_bound);
      }
    }
  }
}

std::vector<ReferencePoint> MPTOptimizer::extendViolatedBounds(
  const std::vector<ReferencePoint> & ref_points) const
{
  auto extended_ref_points = ref_points;
  const int max_length_idx = std::floor(
    mpt_param_.max_longitudinal_margin_for_bound_violation / mpt_param_.delta_arc_length);
  for (int i = 0; i < static_cast<int>(ref_points.size()) - 1; ++i) {
    // before violation
    if (
      ref_points.at(i).bounds.lower_bound <= 0.0 &&
      0.0 <= ref_points.at(i + 1).bounds.lower_bound) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i - j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.lower_bound = ref_points.at(i + 1).bounds.lower_bound;
      }
    }

    if (
      0.0 <= ref_points.at(i).bounds.upper_bound &&
      ref_points.at(i + 1).bounds.upper_bound <= 0.0) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i - j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.upper_bound = ref_points.at(i + 1).bounds.upper_bound;
      }
    }

    // after violation
    if (0 <= ref_points.at(i).bounds.lower_bound && ref_points.at(i + 1).bounds.lower_bound <= 0) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i + j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.lower_bound = ref_points.at(i).bounds.lower_bound;
      }
    }

    if (
      ref_points.at(i).bounds.upper_bound <= 0.0 &&
      0.0 <= ref_points.at(i + 1).bounds.upper_bound) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i + j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.upper_bound = ref_points.at(i).bounds.upper_bound;
      }
    }
  }

  return extended_ref_points;
}

void MPTOptimizer::avoidSuddenSteering(
  std::vector<ReferencePoint> & ref_points, const geometry_msgs::msg::Pose & ego_pose,
  const double ego_vel) const
{
  if (!prev_ref_points_ptr_) {
    return;
  }
  const size_t prev_ego_idx = trajectory_utils::findEgoIndex(
    *prev_ref_points_ptr_, autoware_utils::get_pose(ref_points.front()), ego_nearest_param_);

  const double max_bound_fixing_length = ego_vel * mpt_param_.max_bound_fixing_time;
  const int max_bound_fixing_idx =
    std::floor(max_bound_fixing_length / mpt_param_.delta_arc_length);

  const size_t ego_idx = trajectory_utils::findEgoIndex(ref_points, ego_pose, ego_nearest_param_);
  const size_t max_fixed_bound_idx =
    std::min(ego_idx + static_cast<size_t>(max_bound_fixing_idx), ref_points.size());

  for (size_t i = 0; i < max_fixed_bound_idx; ++i) {
    const size_t prev_idx = std::min(
      prev_ego_idx + i, static_cast<size_t>(static_cast<int>(prev_ref_points_ptr_->size()) - 1));
    const auto & prev_bounds = prev_ref_points_ptr_->at(prev_idx).bounds;

    ref_points.at(i).bounds.upper_bound = prev_bounds.upper_bound;
    ref_points.at(i).bounds.lower_bound = prev_bounds.lower_bound;
  }
}

void MPTOptimizer::updateVehicleBounds(
  std::vector<ReferencePoint> & ref_points,
  const autoware::interpolation::SplineInterpolationPoints2d & ref_points_spline) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  for (size_t p_idx = 0; p_idx < ref_points.size(); ++p_idx) {
    const auto & ref_point = ref_points.at(p_idx);
    // NOTE: This clear is required.
    // It seems they sometimes already have previous values.
    ref_points.at(p_idx).bounds_on_constraints.clear();
    ref_points.at(p_idx).beta.clear();

    for (const double lon_offset : vehicle_circle_longitudinal_offsets_) {
      const auto collision_check_pose =
        ref_points_spline.getSplineInterpolatedPose(p_idx, lon_offset);
      const double collision_check_yaw = tf2::getYaw(collision_check_pose.orientation);

      // calculate beta
      const double beta = ref_point.getYaw() - collision_check_yaw;
      ref_points.at(p_idx).beta.push_back(beta);

      // calculate vehicle_bounds_pose
      const double tmp_yaw = std::atan2(
        collision_check_pose.position.y - ref_point.pose.position.y,
        collision_check_pose.position.x - ref_point.pose.position.x);
      const double offset_y = -autoware_utils::calc_distance2d(ref_point, collision_check_pose) *
                              std::sin(tmp_yaw - collision_check_yaw);

      const auto vehicle_bounds_pose =
        autoware_utils::calc_offset_pose(collision_check_pose, 0.0, offset_y, 0.0);

      // interpolate bounds
      const auto bounds = [&]() {
        const double collision_check_s = ref_points_spline.getAccumulatedLength(p_idx) + lon_offset;
        const size_t collision_check_idx = ref_points_spline.getOffsetIndex(p_idx, lon_offset);

        const size_t prev_idx = std::clamp(
          collision_check_idx - 1, static_cast<size_t>(0),
          static_cast<size_t>(ref_points_spline.getSize() - 2));
        const size_t next_idx = prev_idx + 1;

        const auto & prev_bounds = ref_points.at(prev_idx).bounds;
        const auto & next_bounds = ref_points.at(next_idx).bounds;

        const double prev_s = ref_points_spline.getAccumulatedLength(prev_idx);
        const double next_s = ref_points_spline.getAccumulatedLength(next_idx);

        const double ratio = std::clamp((collision_check_s - prev_s) / (next_s - prev_s), 0.0, 1.0);

        auto bounds = Bounds::lerp(prev_bounds, next_bounds, ratio);
        bounds.translate(offset_y);
        return bounds;
      }();

      ref_points.at(p_idx).bounds_on_constraints.push_back(bounds);
      ref_points.at(p_idx).pose_on_constraints.push_back(vehicle_bounds_pose);
    }
  }
}

// cost function: J = x' Q x + u' R u
MPTOptimizer::ValueMatrix MPTOptimizer::calcValueMatrix(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;

  const bool is_goal_contained = geometry_utils::isSamePoint(ref_points.back(), traj_points.back());

  // update Q
  std::vector<Eigen::Triplet<double>> Q_triplet_vec;
  for (size_t i = 0; i < N_ref; ++i) {
    const auto adaptive_error_weight = [&]() -> std::array<double, 2> {
      // for terminal point
      if (i == N_ref - 1) {
        if (is_goal_contained) {
          return {mpt_param_.goal_lat_error_weight, mpt_param_.goal_yaw_error_weight};
        }
        return {mpt_param_.terminal_lat_error_weight, mpt_param_.terminal_yaw_error_weight};
      }
      // for avoidance
      if (0 < ref_points.at(i).normalized_avoidance_cost) {
        const double lat_error_weight = autoware::interpolation::lerp(
          mpt_param_.lat_error_weight, mpt_param_.avoidance_lat_error_weight,
          ref_points.at(i).normalized_avoidance_cost);
        const double yaw_error_weight = autoware::interpolation::lerp(
          mpt_param_.yaw_error_weight, mpt_param_.avoidance_yaw_error_weight,
          ref_points.at(i).normalized_avoidance_cost);
        return {lat_error_weight, yaw_error_weight};
      }
      // normal case
      return {mpt_param_.lat_error_weight, mpt_param_.yaw_error_weight};
    }();

    const double adaptive_lat_error_weight = adaptive_error_weight.at(0);
    const double adaptive_yaw_error_weight = adaptive_error_weight.at(1);

    Q_triplet_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, adaptive_lat_error_weight));
    Q_triplet_vec.push_back(
      Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, adaptive_yaw_error_weight));
  }
  Eigen::SparseMatrix<double> Q_sparse_mat(N_x, N_x);
  Q_sparse_mat.setFromTriplets(Q_triplet_vec.begin(), Q_triplet_vec.end());

  // update R
  std::vector<Eigen::Triplet<double>> R_triplet_vec;
  for (size_t i = 0; i < N_ref - 1; ++i) {
    const double adaptive_steer_weight = autoware::interpolation::lerp(
      mpt_param_.steer_input_weight, mpt_param_.avoidance_steer_input_weight,
      ref_points.at(i).normalized_avoidance_cost);
    R_triplet_vec.push_back(Eigen::Triplet<double>(D_u * i, D_u * i, adaptive_steer_weight));
  }
  Eigen::SparseMatrix<double> R_sparse_mat(N_u, N_u);
  addSteerWeightR(R_triplet_vec, ref_points);

  R_sparse_mat.setFromTriplets(R_triplet_vec.begin(), R_triplet_vec.end());

  return ValueMatrix{Q_sparse_mat, R_sparse_mat};
}

MPTOptimizer::ObjectiveMatrix MPTOptimizer::calcObjectiveMatrix(
  [[maybe_unused]] const StateEquationGenerator::Matrix & mpt_mat, const ValueMatrix & val_mat,
  const std::vector<ReferencePoint> & ref_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_slack = getNumberOfSlackVariables();

  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_s = N_ref * N_slack;

  const size_t N_v = N_x + N_u + N_s;

  // generate T matrix and vector to shift optimization center
  // NOTE: Z is defined as time-series vector of shifted deviation
  //       error where Z = sparse_T_mat * X + T_vec
  std::vector<Eigen::Triplet<double>> triplet_T_vec;
  Eigen::VectorXd T_vec = Eigen::VectorXd::Zero(N_x);
  const double offset = mpt_param_.optimization_center_offset;
  for (size_t i = 0; i < N_ref; ++i) {
    const double alpha = ref_points.at(i).alpha;

    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, std::cos(alpha)));
    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x + 1, offset * std::cos(alpha)));
    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, 1.0));

    T_vec(i * D_x) = -offset * std::sin(alpha);
  }
  Eigen::SparseMatrix<double> sparse_T_mat(N_x, N_x);
  sparse_T_mat.setFromTriplets(triplet_T_vec.begin(), triplet_T_vec.end());

  // NOTE: min J(v) = min (v'Hv + v'g)
  Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(N_x, N_x);
  H_x.triangularView<Eigen::Upper>() =
    Eigen::MatrixXd(sparse_T_mat.transpose() * val_mat.Q * sparse_T_mat);
  H_x.triangularView<Eigen::Lower>() = H_x.transpose();

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(N_v, N_v);
  H.block(0, 0, N_x, N_x) = H_x;
  H.block(N_x, N_x, N_u, N_u) = val_mat.R;

  Eigen::VectorXd g = Eigen::VectorXd::Zero(N_v);
  g.segment(0, N_x) = T_vec.transpose() * val_mat.Q * sparse_T_mat;
  g.segment(N_x + N_u, N_s) = mpt_param_.soft_collision_free_weight * Eigen::VectorXd::Ones(N_s);

  ObjectiveMatrix obj_matrix;
  obj_matrix.hessian = H;
  obj_matrix.gradient = g;

  return obj_matrix;
}

// Constraint: lb <= A u <= ub
// decision variable
// u := [initial state, steer angles, soft variables]
MPTOptimizer::ConstraintMatrix MPTOptimizer::calcConstraintMatrix(
  const StateEquationGenerator::Matrix & mpt_mat,
  const std::vector<ReferencePoint> & ref_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;

  // NOTE: The number of one-step slack variables.
  //       The number of all slack variables will be N_ref * N_slack.
  const size_t N_slack = getNumberOfSlackVariables();

  const size_t N_v = N_x + N_u + (mpt_param_.soft_constraint ? N_ref * N_slack : 0);

  const size_t N_collision_check = vehicle_circle_longitudinal_offsets_.size();

  // calculate indices of fixed points
  std::vector<size_t> fixed_points_indices;
  for (size_t i = 0; i < N_ref; ++i) {
    if (ref_points.at(i).fixed_kinematic_state) {
      fixed_points_indices.push_back(i);
    }
  }

  // calculate rows and cols of A
  size_t A_rows = 0;
  A_rows += N_x;
  if (mpt_param_.soft_constraint) {
    // NOTE: 3 means expecting slack variable constraints to be larger than lower bound,
    //       smaller than upper bound, and positive.
    A_rows += 3 * N_ref * N_collision_check;
  }
  if (mpt_param_.hard_constraint) {
    A_rows += N_ref * N_collision_check;
  }
  A_rows += fixed_points_indices.size() * D_x;
  if (mpt_param_.steer_limit_constraint) {
    A_rows += N_u;
  }

  // NOTE: The following takes 1 [ms]
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(A_rows, N_v);
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(A_rows, -autoware::osqp_interface::INF);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(A_rows, autoware::osqp_interface::INF);
  size_t A_rows_end = 0;

  // 1. State equation
  A.block(0, 0, N_x, N_x) = Eigen::MatrixXd::Identity(N_x, N_x) - mpt_mat.A;
  A.block(0, N_x, N_x, N_u) = -mpt_mat.B;
  lb.segment(0, N_x) = mpt_mat.W;
  ub.segment(0, N_x) = mpt_mat.W;
  A_rows_end += N_x;

  // 2. collision free
  // CX = C(Bv + w) + C \in R^{N_ref, N_ref * D_x}
  for (size_t l_idx = 0; l_idx < N_collision_check; ++l_idx) {
    // create C := [cos(beta) | l cos(beta)]
    Eigen::SparseMatrix<double> C_sparse_mat(N_ref, N_x);
    std::vector<Eigen::Triplet<double>> C_triplet_vec;
    Eigen::VectorXd C_vec = Eigen::VectorXd::Zero(N_ref);

    // calculate C mat and vec
    for (size_t i = 0; i < N_ref; ++i) {
      const double beta = ref_points.at(i).beta.at(l_idx);
      const double lon_offset = vehicle_circle_longitudinal_offsets_.at(l_idx);

      C_triplet_vec.push_back(Eigen::Triplet<double>(i, i * D_x, 1.0 * std::cos(beta)));
      C_triplet_vec.push_back(Eigen::Triplet<double>(i, i * D_x + 1, lon_offset * std::cos(beta)));
      C_vec(i) = lon_offset * std::sin(beta);
    }
    C_sparse_mat.setFromTriplets(C_triplet_vec.begin(), C_triplet_vec.end());

    // calculate bounds
    const double bounds_offset =
      vehicle_info_.vehicle_width_m / 2.0 - vehicle_circle_radiuses_.at(l_idx);
    const auto & [part_ub, part_lb] = extractBounds(ref_points, l_idx, bounds_offset);

    // soft constraints
    if (mpt_param_.soft_constraint) {
      const size_t A_blk_rows = 3 * N_ref;

      // A := [C | O | ... | O | I | O | ...
      //      -C | O | ... | O | I | O | ...
      //          O    | O | ... | O | I | O | ... ]
      Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(A_blk_rows, N_v);
      A_blk.block(0, 0, N_ref, N_x) = C_sparse_mat;
      A_blk.block(N_ref, 0, N_ref, N_x) = -C_sparse_mat;

      const size_t local_A_offset_cols = N_x + N_u + (!mpt_param_.l_inf_norm ? N_ref * l_idx : 0);
      A_blk.block(0, local_A_offset_cols, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
      A_blk.block(N_ref, local_A_offset_cols, N_ref, N_ref) =
        Eigen::MatrixXd::Identity(N_ref, N_ref);
      A_blk.block(2 * N_ref, local_A_offset_cols, N_ref, N_ref) =
        Eigen::MatrixXd::Identity(N_ref, N_ref);

      // lb := [lower_bound - C
      //        C - upper_bound
      //               O        ]
      Eigen::VectorXd lb_blk = Eigen::VectorXd::Zero(A_blk_rows);
      lb_blk.segment(0, N_ref) = -C_vec + part_lb;
      lb_blk.segment(N_ref, N_ref) = C_vec - part_ub;

      A.block(A_rows_end, 0, A_blk_rows, N_v) = A_blk;
      lb.segment(A_rows_end, A_blk_rows) = lb_blk;

      A_rows_end += A_blk_rows;
    }

    // hard constraints
    if (mpt_param_.hard_constraint) {
      const size_t A_blk_rows = N_ref;

      Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(A_blk_rows, N_v);
      A_blk.block(0, 0, N_ref, N_ref) = C_sparse_mat;

      A.block(A_rows_end, 0, A_blk_rows, N_v) = A_blk;
      lb.segment(A_rows_end, A_blk_rows) = part_lb - C_vec;
      ub.segment(A_rows_end, A_blk_rows) = part_ub - C_vec;

      A_rows_end += A_blk_rows;
    }
  }

  // 3. fixed points constraint
  // X = B v + w where point is fixed
  for (const size_t i : fixed_points_indices) {
    A.block(A_rows_end, D_x * i, D_x, D_x) = Eigen::MatrixXd::Identity(D_x, D_x);

    lb.segment(A_rows_end, D_x) = ref_points.at(i).fixed_kinematic_state->toEigenVector();
    ub.segment(A_rows_end, D_x) = ref_points.at(i).fixed_kinematic_state->toEigenVector();

    A_rows_end += D_x;
  }

  // 4. steer angle limit
  if (mpt_param_.steer_limit_constraint) {
    A.block(A_rows_end, N_x, N_u, N_u) = Eigen::MatrixXd::Identity(N_u, N_u);

    // TODO(murooka) use curvature by stabling optimization
    // Currently, when using curvature, the optimization result is weird with sample_map.
    // lb.segment(A_rows_end, N_u) = Eigen::MatrixXd::Constant(N_u, 1, -mpt_param_.max_steer_rad);
    // ub.segment(A_rows_end, N_u) = Eigen::MatrixXd::Constant(N_u, 1, mpt_param_.max_steer_rad);

    for (size_t i = 0; i < N_u; ++i) {
      const double ref_steer_angle =
        std::atan2(vehicle_info_.wheel_base_m * ref_points.at(i).curvature, 1.0);
      lb(A_rows_end + i) = ref_steer_angle - mpt_param_.max_steer_rad;
      ub(A_rows_end + i) = ref_steer_angle + mpt_param_.max_steer_rad;
    }

    // cppcheck-suppress unreadVariable
    A_rows_end += N_u;
  }

  return ConstraintMatrix{A, lb, ub};
}

void MPTOptimizer::addSteerWeightR(
  std::vector<Eigen::Triplet<double>> & R_triplet_vec,
  const std::vector<ReferencePoint> & ref_points) const
{
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_u = (N_ref - 1) * D_u;

  // add steering rate : weight for (u(i) - u(i-1))^2
  for (size_t i = 0; i < N_u - 1; ++i) {
    R_triplet_vec.push_back(Eigen::Triplet<double>(i, i, mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i + 1, i, -mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i, i + 1, -mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i + 1, i + 1, mpt_param_.steer_rate_weight));
  }
}

std::optional<Eigen::VectorXd> MPTOptimizer::calcOptimizedSteerAngles(
  const std::vector<ReferencePoint> & ref_points, const ObjectiveMatrix & obj_mat,
  const ConstraintMatrix & const_mat)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_slack = getNumberOfSlackVariables();
  const size_t N_v = N_x + N_u + N_ref * N_slack;

  // for manual warm start, calculate initial solution
  const auto u0 = [&]() -> std::optional<Eigen::VectorXd> {
    if (mpt_param_.enable_manual_warm_start) {
      if (prev_ref_points_ptr_ && 1 < prev_ref_points_ptr_->size()) {
        return calcInitialSolutionForManualWarmStart(ref_points, *prev_ref_points_ptr_);
      }
    }
    return std::nullopt;
  }();

  // for manual start, update objective and constraint matrix
  const auto [updated_obj_mat, updated_const_mat] =
    updateMatrixForManualWarmStart(obj_mat, const_mat, u0);

  // calculate matrices for qp
  const Eigen::MatrixXd & H = updated_obj_mat.hessian;
  const Eigen::MatrixXd & A = updated_const_mat.linear;
  const auto f = toStdVector(updated_obj_mat.gradient);
  const auto upper_bound = toStdVector(updated_const_mat.upper_bound);
  const auto lower_bound = toStdVector(updated_const_mat.lower_bound);

  // initialize or update solver according to warm start
  time_keeper_->start_track("initOsqp");

  const autoware::osqp_interface::CSC_Matrix P_csc =
    autoware::osqp_interface::calCSCMatrixTrapezoidal(H);
  const autoware::osqp_interface::CSC_Matrix A_csc = autoware::osqp_interface::calCSCMatrix(A);
  if (
    prev_solution_status_ == 1 && mpt_param_.enable_warm_start && prev_mat_n_ == H.rows() &&
    prev_mat_m_ == A.rows()) {
    RCLCPP_INFO_EXPRESSION(logger_, enable_debug_info_, "warm start");
    osqp_solver_ptr_->updateCscP(P_csc);
    osqp_solver_ptr_->updateQ(f);
    osqp_solver_ptr_->updateCscA(A_csc);
    osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
  } else {
    RCLCPP_INFO_EXPRESSION(logger_, enable_debug_info_, "no warm start");
    osqp_solver_ptr_ = std::make_unique<autoware::osqp_interface::OSQPInterface>(
      P_csc, A_csc, f, lower_bound, upper_bound, osqp_epsilon_);
  }
  prev_mat_n_ = H.rows();
  prev_mat_m_ = A.rows();
  time_keeper_->end_track("initOsqp");

  // solve qp
  time_keeper_->start_track("solveOsqp");
  const autoware::osqp_interface::OSQPResult osqp_result = osqp_solver_ptr_->optimize();
  time_keeper_->end_track("solveOsqp");

  // check solution status
  const int solution_status = osqp_result.solution_status;
  prev_solution_status_ = solution_status;
  if (solution_status != 1) {
    osqp_solver_ptr_->logUnsolvedStatus("[MPT]");
    return std::nullopt;
  }

  // print iteration
  const int iteration_status = osqp_result.iteration_status;
  RCLCPP_INFO_EXPRESSION(logger_, enable_debug_info_, "iteration: %d", iteration_status);

  // get optimization result
  auto optimization_result =
    osqp_result.primal_solution;  // NOTE: const cannot be added due to the next operation.

  const auto has_nan = std::any_of(
    optimization_result.begin(), optimization_result.end(),
    [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    return std::nullopt;
  }
  const Eigen::VectorXd optimized_variables =
    Eigen::Map<Eigen::VectorXd>(&optimization_result[0], N_v);

  if (u0) {  // manual warm start
    return static_cast<Eigen::VectorXd>(optimized_variables + *u0);
  }
  return optimized_variables;
}

Eigen::VectorXd MPTOptimizer::calcInitialSolutionForManualWarmStart(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<ReferencePoint> & prev_ref_points) const
{
  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();
  const size_t N_ref = ref_points.size();
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t D_v = D_x + N_u;
  const size_t N_slack = getNumberOfSlackVariables();
  const size_t D_un = D_v + N_ref * N_slack;

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(D_un);

  const size_t nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    prev_ref_points, ref_points.front().pose, ego_nearest_param_.dist_threshold,
    ego_nearest_param_.yaw_threshold);

  // set previous lateral and yaw deviation
  u0(0) = prev_ref_points.at(nearest_idx).optimized_kinematic_state.lat;
  u0(1) = prev_ref_points.at(nearest_idx).optimized_kinematic_state.yaw;

  // set previous steer angles
  for (size_t i = 0; i < N_u; ++i) {
    const size_t prev_target_idx = std::min(nearest_idx + i, prev_ref_points.size() - 1);
    u0(D_x + i) = prev_ref_points.at(prev_target_idx).optimized_input;
  }

  // set previous slack variables
  for (size_t i = 0; i < N_ref; ++i) {
    const auto & slack_variables = ref_points.at(i).slack_variables;
    if (slack_variables) {
      for (size_t j = 0; j < slack_variables->size(); ++j) {
        u0(D_v + i * N_slack + j) = slack_variables->at(j);
      }
    }
  }

  return u0;
}

std::pair<MPTOptimizer::ObjectiveMatrix, MPTOptimizer::ConstraintMatrix>
MPTOptimizer::updateMatrixForManualWarmStart(
  const ObjectiveMatrix & obj_mat, const ConstraintMatrix & const_mat,
  const std::optional<Eigen::VectorXd> & u0) const
{
  if (!u0) {
    // not manual warm start
    return {obj_mat, const_mat};
  }

  const Eigen::MatrixXd & H = obj_mat.hessian;
  const Eigen::MatrixXd & A = const_mat.linear;

  auto updated_obj_mat = obj_mat;
  auto updated_const_mat = const_mat;

  Eigen::VectorXd & f = updated_obj_mat.gradient;
  Eigen::VectorXd & ub = updated_const_mat.upper_bound;
  Eigen::VectorXd & lb = updated_const_mat.lower_bound;

  // update gradient
  f += H * *u0;

  // update upper_bound and lower_bound
  const Eigen::VectorXd A_times_u0 = A * *u0;
  ub -= A_times_u0;
  lb -= A_times_u0;

  return {updated_obj_mat, updated_const_mat};
}

std::optional<std::vector<TrajectoryPoint>> MPTOptimizer::calcMPTPoints(
  std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & optimized_variables,
  [[maybe_unused]] const StateEquationGenerator::Matrix & mpt_mat) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_slack = getNumberOfSlackVariables();

  const Eigen::VectorXd states = optimized_variables.segment(0, N_x);
  const Eigen::VectorXd steer_angles = optimized_variables.segment(N_x, N_u);
  const Eigen::VectorXd slack_variables = optimized_variables.segment(N_x + N_u, N_ref * N_slack);

  // calculate trajectory points from optimization result
  std::vector<TrajectoryPoint> traj_points;
  for (size_t i = 0; i < N_ref; ++i) {
    auto & ref_point = ref_points.at(i);

    const double lat_error = states(i * D_x);
    const double yaw_error = states(i * D_x + 1);

    // validate optimization result
    if (mpt_param_.enable_optimization_validation) {
      if (
        mpt_param_.max_validation_lat_error < std::abs(lat_error) ||
        mpt_param_.max_validation_yaw_error < std::abs(yaw_error)) {
        return std::nullopt;
      }
    }

    // memorize optimization result (optimized_kinematic_state and optimized_input)
    ref_point.optimized_kinematic_state = KinematicState{lat_error, yaw_error};
    if (i == N_ref - 1) {
      ref_point.optimized_input = 0.0;
    } else {
      ref_point.optimized_input = steer_angles(i * D_u);
    }

    std::vector<double> tmp_slack_variables;
    for (size_t j = 0; j < N_slack; ++j) {
      tmp_slack_variables.push_back(slack_variables(i * N_slack + j));
    }
    ref_point.slack_variables = tmp_slack_variables;

    // update pose and velocity
    TrajectoryPoint traj_point;
    traj_point.pose = ref_point.offsetDeviation(lat_error, yaw_error);
    traj_point.longitudinal_velocity_mps = ref_point.longitudinal_velocity_mps;

    traj_points.push_back(traj_point);
  }

  return traj_points;
}

void MPTOptimizer::publishDebugTrajectories(
  const std_msgs::msg::Header & header, const std::vector<ReferencePoint> & ref_points,
  const std::vector<TrajectoryPoint> & mpt_traj_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // reference points
  const auto ref_traj = autoware::motion_utils::convertToTrajectory(
    trajectory_utils::convertToTrajectoryPoints(ref_points), header);
  debug_ref_traj_pub_->publish(ref_traj);

  // fixed reference points
  const auto fixed_traj_points = extractFixedPoints(ref_points);
  const auto fixed_traj = autoware::motion_utils::convertToTrajectory(fixed_traj_points, header);
  debug_fixed_traj_pub_->publish(fixed_traj);

  // mpt points
  const auto mpt_traj = autoware::motion_utils::convertToTrajectory(mpt_traj_points, header);
  debug_mpt_traj_pub_->publish(mpt_traj);
}

std::vector<TrajectoryPoint> MPTOptimizer::extractFixedPoints(
  const std::vector<ReferencePoint> & ref_points) const
{
  std::vector<TrajectoryPoint> fixed_traj_points;
  for (const auto & ref_point : ref_points) {
    if (ref_point.fixed_kinematic_state) {
      TrajectoryPoint fixed_traj_point;
      fixed_traj_point.pose = ref_point.offsetDeviation(
        ref_point.fixed_kinematic_state->lat, ref_point.fixed_kinematic_state->yaw);
      fixed_traj_points.push_back(fixed_traj_point);
    }
  }

  return fixed_traj_points;
}

double MPTOptimizer::getTrajectoryLength() const
{
  const double forward_traj_length = mpt_param_.num_points * mpt_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;
  return forward_traj_length + backward_traj_length;
}

double MPTOptimizer::getDeltaArcLength() const
{
  return mpt_param_.delta_arc_length;
}

int MPTOptimizer::getNumberOfPoints() const
{
  return mpt_param_.num_points;
}

size_t MPTOptimizer::getNumberOfSlackVariables() const
{
  if (mpt_param_.soft_constraint) {
    if (mpt_param_.l_inf_norm) {
      return 1;
    }
    return vehicle_circle_longitudinal_offsets_.size();
  }
  return 0;
}

std::optional<double> MPTOptimizer::calcNormalizedAvoidanceCost(
  const ReferencePoint & ref_point) const
{
  const double negative_avoidance_cost = std::min(
    -ref_point.bounds.lower_bound - mpt_param_.avoidance_cost_margin,
    ref_point.bounds.upper_bound - mpt_param_.avoidance_cost_margin);
  if (0 <= negative_avoidance_cost) {
    return {};
  }
  return std::clamp(-negative_avoidance_cost / mpt_param_.max_avoidance_cost, 0.0, 1.0);
}
}  // namespace autoware::path_optimizer
