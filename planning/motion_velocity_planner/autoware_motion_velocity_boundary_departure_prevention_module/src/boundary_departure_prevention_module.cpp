// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "boundary_departure_prevention_module.hpp"

#include "debug.hpp"
#include "slow_down_interpolator.hpp"
#include "utils.hpp"

#include <autoware/boundary_departure_checker/data_structs.hpp>
#include <autoware/boundary_departure_checker/parameters.hpp>
#include <autoware/boundary_departure_checker/utils.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <magic_enum.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/view.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using autoware::motion_velocity_planner::experimental::DepartureType;
std::string to_string(DepartureType type)
{
  auto str = std::string(magic_enum::enum_name(type));
  std::transform(
    str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::tolower(c); });
  std::replace(str.begin(), str.end(), '_', ' ');
  return str;
}
}  // namespace

namespace autoware::motion_velocity_planner::experimental
{

void BoundaryDeparturePreventionModule::init(
  rclcpp::Node & node, [[maybe_unused]] const std::string & module_name)
{
  module_name_ = module_name;
  clock_ptr_ = node.get_clock();
  logger_ = node.get_logger();
  node_param_ = NodeParam(node);

  subscribe_topics(node);
  publish_topics(node);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(processing_time_detail_pub_);

  updater_ptr_ = std::make_unique<diagnostic_updater::Updater>(&node);
  updater_ptr_->setHardwareID("motion_velocity_boundary_departure_prevention");
  updater_ptr_->add(
    "boundary_departure", [this](diagnostic_updater::DiagnosticStatusWrapper & stat) {
      const auto [lvl, msg] = output_.diag_status;
      stat.summary(lvl, msg);
    });

  last_abnormality_fp_no_overlap_bound_time_ = clock_ptr_->now().seconds();
  last_abnormality_fp_overlap_bound_time_ = clock_ptr_->now().seconds();
  last_no_critical_dpt_time_ = clock_ptr_->now().seconds();
}

void BoundaryDeparturePreventionModule::update_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  using autoware_utils::update_param;
  auto & pp = node_param_;
  auto & configs = pp.bdc_param.abnormality_configs;

  const std::string module_name{"boundary_departure_prevention."};
  update_param(
    parameters, module_name + "boundary_types_to_detect", pp.bdc_param.boundary_types_to_detect);
  update_param(
    parameters, module_name + "th_max_lateral_query_num", pp.bdc_param.th_max_lateral_query_num);

  const std::string ns_abnormality{module_name + "abnormality."};
  const std::string ns_normal_abnormality{ns_abnormality + "normal."};
  const std::string ns_localization_abnormality{ns_abnormality + "localization."};
  const std::string ns_longitudinal_abnormality{ns_abnormality + "longitudinal."};

  const auto has_type = [&](const AbnormalityType type_to_check) {
    return std::any_of(
      pp.bdc_param.abnormality_types_to_compensate.begin(),
      pp.bdc_param.abnormality_types_to_compensate.end(),
      [&](const AbnormalityType type) { return type == type_to_check; });
  };

  const auto update_types_to_compensate = [&](const AbnormalityType type, const std::string & ns) {
    const auto original_compensate = has_type(type);
    bool new_compensate{};
    if (update_param(parameters, ns + "enable", new_compensate)) {
      if (new_compensate && !original_compensate) {
        pp.bdc_param.abnormality_types_to_compensate.push_back(type);
      } else if (!new_compensate && original_compensate) {
        pp.bdc_param.abnormality_types_to_compensate.erase(
          std::remove(
            pp.bdc_param.abnormality_types_to_compensate.begin(),
            pp.bdc_param.abnormality_types_to_compensate.end(), type),
          pp.bdc_param.abnormality_types_to_compensate.end());
      }
    }
  };
  update_types_to_compensate(AbnormalityType::NORMAL, ns_normal_abnormality);
  NormalConfig normal_config =
    pp.bdc_param.get_abnormality_config<NormalConfig>(AbnormalityType::NORMAL).value();
  const std::string footprint_envelop_ns{ns_normal_abnormality + "footprint_envelop."};
  update_param(parameters, footprint_envelop_ns + "lat_m", normal_config.footprint_envelop.lat_m);
  update_param(parameters, footprint_envelop_ns + "lon_m", normal_config.footprint_envelop.lon_m);
  configs.insert_or_assign(AbnormalityType::NORMAL, normal_config);

  const auto update_steer_params = [&](const auto steer_abnormality_type, const auto & ns) {
    update_types_to_compensate(steer_abnormality_type, ns);
    SteeringConfig steering_config =
      pp.bdc_param.get_abnormality_config<SteeringConfig>(steer_abnormality_type).value();
    update_param(parameters, ns + "delay_s", steering_config.delay_s);
    update_param(parameters, ns + "factor", steering_config.factor);
    update_param(parameters, ns + "offset_rps", steering_config.offset_rps);
    update_param(
      parameters, ns + "steering_rate_velocities_mps",
      steering_config.steering_rate_velocities_mps);
    update_param(
      parameters, ns + "steering_rate_limits_rps", steering_config.steering_rate_limits_rps);
    configs.insert_or_assign(steer_abnormality_type, steering_config);
  };
  const std::string ns_steering_abnormality_accelerated{ns_abnormality + "steering_accelerated."};
  update_steer_params(AbnormalityType::STEERING_ACCELERATED, ns_steering_abnormality_accelerated);
  const std::string ns_steering_abnormality_stuck{ns_abnormality + "steering_stuck."};
  update_steer_params(AbnormalityType::STEERING_STUCK, ns_steering_abnormality_stuck);
  const std::string ns_steering_abnormality_sudden_left{ns_abnormality + "steering_sudden_left."};
  update_steer_params(AbnormalityType::STEERING_SUDDEN_LEFT, ns_steering_abnormality_sudden_left);
  const std::string ns_steering_abnormality_sudden_right{ns_abnormality + "steering_sudden_right."};
  update_steer_params(AbnormalityType::STEERING_SUDDEN_RIGHT, ns_steering_abnormality_sudden_right);

  update_types_to_compensate(AbnormalityType::LOCALIZATION, ns_localization_abnormality);
  LocalizationConfig localization_config =
    pp.bdc_param.get_abnormality_config<LocalizationConfig>(AbnormalityType::LOCALIZATION).value();
  const std::string localization_footprint_envelop_ns{
    ns_localization_abnormality + "footprint_envelop."};
  update_param(
    parameters, localization_footprint_envelop_ns + "lat_m",
    localization_config.footprint_envelop.lat_m);
  update_param(
    parameters, localization_footprint_envelop_ns + "lon_m",
    localization_config.footprint_envelop.lon_m);
  configs.insert_or_assign(AbnormalityType::LOCALIZATION, localization_config);

  update_types_to_compensate(AbnormalityType::LOCALIZATION, ns_longitudinal_abnormality);
  LongitudinalConfig longitudinal_config =
    pp.bdc_param.get_abnormality_config<LongitudinalConfig>(AbnormalityType::LONGITUDINAL).value();
  const std::string lon_tracking_ns{ns_longitudinal_abnormality + "lon_tracking."};
  update_param(parameters, lon_tracking_ns + "scale", longitudinal_config.lon_tracking.scale);
  update_param(
    parameters, lon_tracking_ns + "extra_margin_m",
    longitudinal_config.lon_tracking.extra_margin_m);
  configs.insert_or_assign(AbnormalityType::LONGITUDINAL, longitudinal_config);

  // Point/goal shift thresholds
  update_param(parameters, module_name + "th_pt_shift.dist_m", pp.th_pt_shift_dist_m);

  auto th_pt_shift_angle_deg = autoware_utils_math::rad2deg(pp.th_pt_shift_angle_rad);
  update_param(parameters, module_name + "th_pt_shift.angle_deg", th_pt_shift_angle_deg);
  pp.th_pt_shift_angle_rad = autoware_utils_math::deg2rad(th_pt_shift_angle_deg);

  update_param(parameters, module_name + "th_pt_shift.goal_dist_m", pp.th_goal_shift_dist_m);

  update_param(
    parameters, module_name + "th_cutoff_time_s.predicted_path",
    pp.bdc_param.th_cutoff_time_predicted_path_s);
  update_param(
    parameters, module_name + "th_cutoff_time_s.near_boundary",
    pp.bdc_param.th_cutoff_time_near_boundary_s);
  update_param(
    parameters, module_name + "th_cutoff_time_s.departure",
    pp.bdc_param.th_cutoff_time_departure_s);

  update_param(
    parameters, module_name + "on_time_buffer_s.critical_departure",
    pp.on_time_buffer_s.critical_departure);
  update_param(
    parameters, module_name + "on_time_buffer_s.near_boundary", pp.on_time_buffer_s.near_boundary);
  update_param(
    parameters, module_name + "off_time_buffer_s.critical_departure",
    pp.off_time_buffer_s.critical_departure);
  update_param(
    parameters, module_name + "off_time_buffer_s.near_boundary",
    pp.off_time_buffer_s.near_boundary);

  const std::string ns_slow_down{module_name + "slow_down_behavior.enable."};

  bool enable_slow_down_near_bound = pp.slow_down_types.count(DepartureType::NEAR_BOUNDARY);
  if (update_param(
        parameters, ns_slow_down + "slow_down_near_boundary", enable_slow_down_near_bound)) {
    if (enable_slow_down_near_bound)
      pp.slow_down_types.insert(DepartureType::NEAR_BOUNDARY);
    else
      pp.slow_down_types.erase(DepartureType::NEAR_BOUNDARY);
  }

  bool enable_approaching_dpt = pp.slow_down_types.count(DepartureType::APPROACHING_DEPARTURE);
  if (update_param(
        parameters, ns_slow_down + "slow_down_before_departure", enable_approaching_dpt)) {
    if (enable_approaching_dpt)
      pp.slow_down_types.insert(DepartureType::APPROACHING_DEPARTURE);
    else
      pp.slow_down_types.erase(DepartureType::APPROACHING_DEPARTURE);
  }

  bool enable_critical_dpt = pp.slow_down_types.count(DepartureType::CRITICAL_DEPARTURE);
  if (update_param(parameters, ns_slow_down + "stop_before_departure", enable_critical_dpt)) {
    if (enable_critical_dpt)
      pp.slow_down_types.insert(DepartureType::CRITICAL_DEPARTURE);
    else
      pp.slow_down_types.erase(DepartureType::CRITICAL_DEPARTURE);
  }

  // Trigger thresholds
  const std::string ns_th_trigger{module_name + "slow_down_behavior.th_trigger."};

  auto th_vel_min_kmph = autoware_utils_math::mps2kmph(pp.bdc_param.th_trigger.th_vel_mps.min);
  update_param(parameters, ns_th_trigger + "th_vel_kmph.min", th_vel_min_kmph);
  pp.bdc_param.th_trigger.th_vel_mps.min = autoware_utils_math::kmph2mps(th_vel_min_kmph);

  auto th_vel_max_kmph = autoware_utils_math::mps2kmph(pp.bdc_param.th_trigger.th_vel_mps.max);
  update_param(parameters, ns_th_trigger + "th_vel_kmph.max", th_vel_max_kmph);
  pp.bdc_param.th_trigger.th_vel_mps.max = autoware_utils_math::kmph2mps(th_vel_max_kmph);

  update_param(
    parameters, ns_th_trigger + "th_acc_mps2.min", pp.bdc_param.th_trigger.th_acc_mps2.min);
  update_param(
    parameters, ns_th_trigger + "th_acc_mps2.max", pp.bdc_param.th_trigger.th_acc_mps2.max);

  update_param(
    parameters, ns_th_trigger + "th_jerk_mps3.min", pp.bdc_param.th_trigger.th_jerk_mps3.min);
  update_param(
    parameters, ns_th_trigger + "th_jerk_mps3.max", pp.bdc_param.th_trigger.th_jerk_mps3.max);

  update_param(parameters, ns_th_trigger + "brake_delay_s", pp.bdc_param.th_trigger.brake_delay_s);
  update_param(parameters, ns_th_trigger + "dist_error_m", pp.bdc_param.th_trigger.dist_error_m);

  const std::string ns_dist_to_bound{ns_th_trigger + "th_dist_to_boundary_m."};
  const std::string ns_dist_to_bound_left{ns_dist_to_bound + "left."};
  const std::string ns_dist_to_bound_right{ns_dist_to_bound + "right."};

  update_param(
    parameters, ns_dist_to_bound_left + "min",
    pp.bdc_param.th_trigger.th_dist_to_boundary_m.left.min);
  update_param(
    parameters, ns_dist_to_bound_left + "max",
    pp.bdc_param.th_trigger.th_dist_to_boundary_m.left.max);
  update_param(
    parameters, ns_dist_to_bound_right + "min",
    pp.bdc_param.th_trigger.th_dist_to_boundary_m.right.min);
  update_param(
    parameters, ns_dist_to_bound_right + "max",
    pp.bdc_param.th_trigger.th_dist_to_boundary_m.right.max);

  // Diagnostic levels
  const auto update_diag = [&](DepartureType type, const std::string & key) {
    auto level = static_cast<int8_t>(pp.diagnostic_level.at(type));
    if (update_param(parameters, module_name + "diagnostic." + key, level)) {
      pp.diagnostic_level[type] = static_cast<int8_t>(level);
    }
  };
  update_diag(DepartureType::NEAR_BOUNDARY, "near_boundary");
  update_diag(DepartureType::APPROACHING_DEPARTURE, "approaching_departure");
  update_diag(DepartureType::CRITICAL_DEPARTURE, "critical_departure");

  if (boundary_departure_checker_ptr_) {
    boundary_departure_checker_ptr_->setParam(pp.bdc_param);
  }
}

void BoundaryDeparturePreventionModule::subscribe_topics(rclcpp::Node & node)
{
  ego_pred_traj_polling_sub_ =
    autoware_utils::InterProcessPollingSubscriber<Trajectory>::create_subscription(
      &node, "/control/trajectory_follower/lateral/predicted_trajectory", 1);
  control_cmd_polling_sub_ =
    autoware_utils::InterProcessPollingSubscriber<Control>::create_subscription(
      &node, "/control/command/control_cmd", 1);
  steering_angle_polling_sub_ =
    autoware_utils::InterProcessPollingSubscriber<SteeringReport>::create_subscription(
      &node, "/vehicle/status/steering_status", 1);
  op_mode_state_polling_sub_ =
    autoware_utils::InterProcessPollingSubscriber<OperationModeState>::create_subscription(
      &node, "/api/operation_mode/state", 1);
  route_polling_sub_ = autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>::
    create_subscription(
      &node, "/planning/mission_planning/route", rclcpp::QoS(1).transient_local());
}

void BoundaryDeparturePreventionModule::publish_topics(rclcpp::Node & node)
{
  const std::string ns = "boundary_departure_prevention";

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns + "/debug_markers", 1);

  virtual_wall_publisher_ = node.create_publisher<MarkerArray>("~/" + ns + "/virtual_walls", 1);

  processing_time_detail_pub_ = node.create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/" + ns, 1);

  processing_time_publisher_ =
    node.create_publisher<Float64Stamped>("~/debug/" + ns + "/processing_time_ms", 1);
}

void BoundaryDeparturePreventionModule::take_data()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (const auto ego_pred_traj_msg = ego_pred_traj_polling_sub_->take_data()) {
    ego_pred_traj_ptr_ = ego_pred_traj_msg;
  }

  if (const auto control_command_msg = control_cmd_polling_sub_->take_data()) {
    control_cmd_ptr_ = control_command_msg;
  }

  if (const auto steering_angle_msg = steering_angle_polling_sub_->take_data()) {
    steering_angle_ptr_ = steering_angle_msg;
  }

  if (const auto op_mode_state_msg = op_mode_state_polling_sub_->take_data()) {
    op_mode_state_ptr_ = op_mode_state_msg;
  }

  if (const auto route_msg = route_polling_sub_->take_data()) {
    route_ptr_ = route_msg;
  }
}

VelocityPlanningResult BoundaryDeparturePreventionModule::plan(
  const TrajectoryPoints & raw_trajectory_points,
  [[maybe_unused]] const TrajectoryPoints & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  StopWatch<std::chrono::milliseconds> stopwatch_ms;
  stopwatch_ms.tic("plan");

  take_data();

  const auto result_opt = plan_velocities(raw_trajectory_points, planner_data);

  if (updater_ptr_) {
    updater_ptr_->force_update();
  }

  if (clock_ptr_ && processing_time_publisher_) {
    processing_time_publisher_->publish(std::invoke([&]() {
      autoware_internal_debug_msgs::msg::Float64Stamped msg;
      msg.stamp = clock_ptr_->now();
      msg.data = stopwatch_ms.toc();
      return msg;
    }));
  }

  if (!result_opt) {
    RCLCPP_DEBUG(logger_, "%s", result_opt.error().c_str());
    return {};
  }

  return *result_opt;
}

tl::expected<VelocityPlanningResult, std::string>
BoundaryDeparturePreventionModule::plan_velocities(
  const TrajectoryPoints & raw_trajectory_points,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (const auto invalid_data_opt = is_data_invalid(raw_trajectory_points)) {
    return tl::make_unexpected(*invalid_data_opt);
  }

  if (const auto is_timeout_opt = is_data_timeout(planner_data->current_odometry)) {
    return tl::make_unexpected(*is_timeout_opt);
  }

  if (const auto is_new_route = is_route_changed()) {
    RCLCPP_WARN(logger_, "%s. Reset output.", is_new_route->c_str());
    output_ = Output();
  }

  if (!is_autonomous_mode()) {
    return tl::make_unexpected("Not in autonomous mode.");
  }

  const auto & vehicle_info = planner_data->vehicle_info_;
  const auto & ll_map_ptr = planner_data->route_handler->getLaneletMapPtr();

  if (!boundary_departure_checker_ptr_) {
    boundary_departure_checker_ptr_ = std::make_unique<BoundaryDepartureChecker>(
      ll_map_ptr, vehicle_info, node_param_.bdc_param, time_keeper_);
  }

  if (!slow_down_interpolator_ptr_) {
    slow_down_interpolator_ptr_ =
      std::make_unique<utils::SlowDownInterpolator>(node_param_.bdc_param.th_trigger);
  }

  try {
    auto result_opt = plan_slow_down_intervals(raw_trajectory_points, planner_data);

    publish_visualization_markers();

    if (!result_opt) {
      return tl::make_unexpected(result_opt.error());
    }

    return *result_opt;
  } catch (const std::exception & e) {
    return tl::make_unexpected(fmt::format("Exception: {}", e.what()));
  } catch (...) {
    return tl::make_unexpected("Unknown exception.");
  }
}

std::optional<std::string> BoundaryDeparturePreventionModule::is_data_invalid(
  const TrajectoryPoints & raw_trajectory_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!ego_pred_traj_ptr_) {
    return {"waiting for predicted trajectory..."};
  }

  if (ego_pred_traj_ptr_->points.empty()) {
    return {"empty predicted trajectory..."};
  }

  if (!op_mode_state_ptr_) {
    return {"waiting for operation mode state..."};
  }

  if (!control_cmd_ptr_) {
    return {"waiting for control command..."};
  }

  if (!steering_angle_ptr_) {
    return {"waiting for steering angle..."};
  }

  if (!route_ptr_) {
    return {"waiting for route..."};
  }

  constexpr size_t min_pts_size = 4;
  if (raw_trajectory_points.size() < min_pts_size) {
    return {"Insufficient reference trajectory points."};
  }

  return std::nullopt;
}

std::optional<std::string> BoundaryDeparturePreventionModule::is_data_timeout(
  const Odometry & odom) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto now = clock_ptr_->now();
  const auto odom_time_diff_s = (rclcpp::Time(odom.header.stamp) - now).seconds();
  constexpr auto th_pose_timeout_s = 1.0;

  if (odom_time_diff_s > th_pose_timeout_s) {
    return fmt::format("pose timeout for {:.3f} seconds...", odom_time_diff_s);
  }

  const auto ego_pred_traj_time_diff_s =
    (rclcpp::Time(ego_pred_traj_ptr_->header.stamp) - now).seconds();

  if (ego_pred_traj_time_diff_s > th_pose_timeout_s) {
    return fmt::format(
      "ego predicted trajectory timeout for {:.3f} seconds...", ego_pred_traj_time_diff_s);
  }

  const auto control_command_time_diff_s = (rclcpp::Time(control_cmd_ptr_->stamp) - now).seconds();
  if (control_command_time_diff_s > th_pose_timeout_s) {
    return fmt::format(
      "control commands timeout for {:.3f} seconds...", control_command_time_diff_s);
  }

  const auto steering_angle_time_diff_s =
    (rclcpp::Time(steering_angle_ptr_->stamp) - now).seconds();
  if (steering_angle_time_diff_s > th_pose_timeout_s) {
    return fmt::format("steering report timeout for {:.3f} seconds...", steering_angle_time_diff_s);
  }

  return std::nullopt;
}

bool BoundaryDeparturePreventionModule::is_autonomous_mode() const
{
  return (op_mode_state_ptr_->mode == OperationModeState::AUTONOMOUS) &&
         op_mode_state_ptr_->is_autoware_control_enabled;
}

std::optional<std::string> BoundaryDeparturePreventionModule::is_route_changed()
{
  if (!prev_route_ptr_) {
    prev_route_ptr_ = std::make_unique<LaneletRoute>(*route_ptr_);
    return fmt::format("Initializing previous route pointer.");
  }

  const auto prev_uuid = autoware_utils::to_boost_uuid(prev_route_ptr_->uuid);
  const auto curr_uuid = autoware_utils::to_boost_uuid(route_ptr_->uuid);

  if (prev_uuid != curr_uuid) {
    *prev_route_ptr_ = *route_ptr_;
    return fmt::format("Route has changed.");
  }

  const auto & prev_goal = prev_route_ptr_->goal_pose;
  const auto & curr_goal = route_ptr_->goal_pose;
  const auto diff_to_new_goal = autoware_utils::calc_distance2d(prev_goal, curr_goal);

  if (diff_to_new_goal > node_param_.th_goal_shift_dist_m) {
    *prev_route_ptr_ = *route_ptr_;
    return fmt::format("Goal changed due to exceeding threshold.");
  }

  return std::nullopt;
}

tl::expected<VelocityPlanningResult, std::string>
BoundaryDeparturePreventionModule::plan_slow_down_intervals(
  const TrajectoryPoints & raw_trajectory_points,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & vehicle_info = planner_data->vehicle_info_;
  const auto & curr_odom = planner_data->current_odometry;
  const auto curr_vel = planner_data->current_odometry.twist.twist.linear.x;
  const auto curr_acc = planner_data->current_acceleration.accel.accel.linear.x;
  const auto & curr_pose = curr_odom.pose;
  const auto & curr_position = curr_pose.pose.position;
  const auto & goal_position = raw_trajectory_points.back().pose.position;
  constexpr auto min_effective_dist = 1.0;

  StopWatch<std::chrono::milliseconds> stopwatch_ms;
  const auto toc_curr_watch = [&](const std::string & tag) {
    processing_times_ms_[tag] = stopwatch_ms.toc(true);
  };

  if (autoware_utils::calc_distance2d(curr_position, goal_position) < min_effective_dist) {
    output_.departure_intervals.clear();
    return tl::make_unexpected("Too close to goal.");
  }
  toc_curr_watch("chk_dist_to_goal");

  const auto ref_traj_pts_opt =
    trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(raw_trajectory_points);

  if (!ref_traj_pts_opt) {
    return tl::make_unexpected(ref_traj_pts_opt.error().what);
  }

  const auto abnormality_data_opt = boundary_departure_checker_ptr_->get_abnormalities_data(
    ego_pred_traj_ptr_->points, *ref_traj_pts_opt, curr_pose, *steering_angle_ptr_);

  if (!abnormality_data_opt) {
    return tl::make_unexpected(abnormality_data_opt.error());
  }

  output_.abnormalities_data = *abnormality_data_opt;
  toc_curr_watch("get_abnormalities_data");

  const auto closest_projections_to_bound_opt =
    boundary_departure_checker_ptr_->get_closest_projections_to_boundaries(
      output_.abnormalities_data.projections_to_bound, curr_vel, curr_acc);
  toc_curr_watch("get_ref_traj");

  if (!closest_projections_to_bound_opt) {
    return tl::make_unexpected(closest_projections_to_bound_opt.error());
  }

  output_.closest_projections_to_bound = *closest_projections_to_bound_opt;

  const auto ego_dist_on_traj_m =
    motion_utils::calcSignedArcLength(raw_trajectory_points, 0UL, curr_pose.pose.position);
  toc_curr_watch("chk_ego_dist_on_traj");

  const auto lon_offset_m = [&vehicle_info](const bool take_front_offset) {
    return take_front_offset ? vehicle_info.max_longitudinal_offset_m
                             : vehicle_info.min_longitudinal_offset_m;
  };
  const auto ego_dist_on_traj_with_offset_m = [&](const bool take_front_offset) {
    return ego_dist_on_traj_m + lon_offset_m(take_front_offset);
  };

  std::vector<double> pred_traj_idx_to_ref_traj_lon_dist;
  pred_traj_idx_to_ref_traj_lon_dist.reserve(ego_pred_traj_ptr_->points.size());
  for (const auto & p : ego_pred_traj_ptr_->points) {
    pred_traj_idx_to_ref_traj_lon_dist.push_back(
      motion_utils::calcSignedArcLength(raw_trajectory_points, 0UL, p.pose.position));
  }
  output_.departure_points = boundary_departure_checker_ptr_->get_departure_points(
    output_.closest_projections_to_bound, pred_traj_idx_to_ref_traj_lon_dist);
  toc_curr_watch("get_departure_points");

  // update output_.critical_departure_points
  update_critical_departure_points(raw_trajectory_points, ego_dist_on_traj_m);
  toc_curr_watch("update_critical_departure_points");

  const auto is_departure_persist = std::invoke([&]() {
    const auto is_found =
      std::any_of(g_side_keys.begin(), g_side_keys.end(), [&](const auto side_key) {
        return !closest_projections_to_bound_opt.value()[side_key].empty();
      });

    if (!is_found) {
      last_abnormality_fp_overlap_bound_time_ = clock_ptr_->now().seconds();
      return false;
    }

    const auto t_diff = clock_ptr_->now().seconds() - last_abnormality_fp_overlap_bound_time_;
    return t_diff >= node_param_.on_time_buffer_s.near_boundary;
  });

  if (output_.departure_intervals.empty() && is_departure_persist) {
    output_.departure_intervals = utils::init_departure_intervals(
      *ref_traj_pts_opt, output_.departure_points,
      ego_dist_on_traj_with_offset_m(!planner_data->is_driving_forward),
      node_param_.slow_down_types);
  }

  if (!output_.departure_intervals.empty()) {
    auto & departure_intervals_mut = output_.departure_intervals;

    const auto is_reset_interval = std::invoke([&]() {
      const auto is_departure_found = std::any_of(
        g_side_keys.begin(), g_side_keys.end(),
        [&](const auto side_key) { return !output_.departure_points[side_key].empty(); });

      if (is_departure_found) {
        last_abnormality_fp_no_overlap_bound_time_ = clock_ptr_->now().seconds();
        return false;
      }
      const auto t_diff = clock_ptr_->now().seconds() - last_abnormality_fp_no_overlap_bound_time_;
      return t_diff >= node_param_.off_time_buffer_s.near_boundary;
    });

    utils::update_departure_intervals(
      departure_intervals_mut, output_.departure_points, *ref_traj_pts_opt,
      vehicle_info.vehicle_length_m, raw_trajectory_points,
      ego_dist_on_traj_with_offset_m(!planner_data->is_driving_forward),
      node_param_.th_pt_shift_dist_m, node_param_.th_pt_shift_angle_rad,
      node_param_.slow_down_types, is_reset_interval, is_departure_persist);

    if (is_reset_interval) {
      last_abnormality_fp_no_overlap_bound_time_ = clock_ptr_->now().seconds();
    }
    const auto reset_lost_time =
      std::any_of(g_side_keys.begin(), g_side_keys.end(), [&](const auto side_key) {
        return output_.departure_intervals.empty() || output_.departure_points[side_key].empty();
      });

    if (reset_lost_time) {
      last_abnormality_fp_overlap_bound_time_ = clock_ptr_->now().seconds();
    }
  }

  toc_curr_watch("update_departure_intervals");

  output_.slowdown_intervals = utils::get_slow_down_intervals(
    *ref_traj_pts_opt, output_.departure_intervals, *slow_down_interpolator_ptr_,
    curr_odom.twist.twist.linear.x, planner_data->current_acceleration.accel.accel.linear.x,
    ego_dist_on_traj_with_offset_m(planner_data->is_driving_forward));

  std::vector<SlowdownInterval> slowdown_intervals;
  for (auto && [idx, slowdown_interval] : output_.slowdown_intervals | ranges::views::enumerate) {
    const auto & [start_pose, end_pose, vel] = slowdown_interval;
    slowdown_intervals.emplace_back(start_pose.position, end_pose.position, vel);
  }
  toc_curr_watch("get_slow_down_interval");

  output_.diag_status = get_diagnostic_status(ego_dist_on_traj_m, curr_vel);

  toc_curr_watch("get_diagnostic_status");

  VelocityPlanningResult result;
  result.slowdown_intervals = slowdown_intervals;
  return result;
}

void BoundaryDeparturePreventionModule::update_critical_departure_points(
  const std::vector<TrajectoryPoint> & raw_ref_traj, const double offset_from_ego)
{
  if (!is_critical_departure_persist()) {
    output_.critical_departure_points.clear();
  }

  for (auto & crit_dpt_pt_mut : output_.critical_departure_points) {
    crit_dpt_pt_mut.ego_dist_on_ref_traj = motion_utils::calcSignedArcLength(
      raw_ref_traj, 0UL, crit_dpt_pt_mut.pose_on_current_ref_traj.position);

    if (crit_dpt_pt_mut.ego_dist_on_ref_traj < offset_from_ego) {
      crit_dpt_pt_mut.can_be_removed = true;
      continue;
    }

    const auto updated_pose =
      motion_utils::calcInterpolatedPose(raw_ref_traj, crit_dpt_pt_mut.ego_dist_on_ref_traj);
    if (
      const auto is_shifted_opt = utils::is_point_shifted(
        crit_dpt_pt_mut.pose_on_current_ref_traj, updated_pose, node_param_.th_pt_shift_dist_m,
        node_param_.th_pt_shift_angle_rad)) {
      crit_dpt_pt_mut.can_be_removed = true;
    }
  }

  utils::remove_if(
    output_.critical_departure_points, [](const DeparturePoint & pt) { return pt.can_be_removed; });

  if (!is_continuous_critical_departure()) {
    return;
  }

  auto new_critical_departure_point = utils::find_new_critical_departure_points(
    output_.departure_points, output_.critical_departure_points, raw_ref_traj,
    node_param_.bdc_param.th_point_merge_distance_m);

  if (new_critical_departure_point.empty()) {
    return;
  }

  std::move(
    new_critical_departure_point.begin(), new_critical_departure_point.end(),
    std::back_inserter(output_.critical_departure_points));

  std::sort(output_.critical_departure_points.begin(), output_.critical_departure_points.end());
}

std::pair<int8_t, std::string> BoundaryDeparturePreventionModule::get_diagnostic_status(
  const double ego_dist_on_traj, const double curr_vel)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto diag_type = std::invoke([&]() {
    const auto is_within_braking_dist = [&](const DeparturePoint & pt) {
      const auto & th_trigger = node_param_.bdc_param.th_trigger;
      const auto braking_start_vel =
        std::clamp(curr_vel, th_trigger.th_vel_mps.min, th_trigger.th_vel_mps.max);
      const auto braking_dist =
        boundary_departure_checker::utils::calc_judge_line_dist_with_jerk_limit(
          braking_start_vel, 0.0, th_trigger.th_acc_mps2.min, th_trigger.th_jerk_mps3.max,
          th_trigger.brake_delay_s);
      return (pt.ego_dist_on_ref_traj - ego_dist_on_traj) <= braking_dist;
    };

    if (ranges::any_of(output_.critical_departure_points, is_within_braking_dist)) {
      return DepartureType::CRITICAL_DEPARTURE;
    }

    const auto has_type = [&](const DepartureType type) {
      const auto is_type = [type](const DeparturePoint & pt) { return pt.departure_type == type; };

      return ranges::any_of(g_side_keys, [&](const SideKey side_key) {
        return ranges::any_of(output_.departure_points[side_key], is_type);
      });
    };

    if (has_type(DepartureType::APPROACHING_DEPARTURE)) {
      return DepartureType::APPROACHING_DEPARTURE;
    }

    if (has_type(DepartureType::NEAR_BOUNDARY)) {
      return DepartureType::NEAR_BOUNDARY;
    }

    return DepartureType::NONE;
  });

  auto lvl = node_param_.diagnostic_level[diag_type];
  auto msg = to_string(diag_type);

  if (lvl != DiagStatus::OK && diag_type != DepartureType::NONE) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_ptr_, throttle_duration_ms, "%s", msg.c_str());
  }

  return {lvl, msg};
}

bool BoundaryDeparturePreventionModule::is_continuous_critical_departure()
{
  const auto is_critical_departure_found =
    std::any_of(g_side_keys.begin(), g_side_keys.end(), [&](const auto side_key) {
      const auto & closest_projections = output_.closest_projections_to_bound[side_key];
      return std::any_of(
        closest_projections.rbegin(), closest_projections.rend(),
        [](const auto & pt) { return pt.departure_type == DepartureType::CRITICAL_DEPARTURE; });
    });

  if (!is_critical_departure_found) {
    last_no_critical_dpt_time_ = clock_ptr_->now().seconds();
    return false;
  }

  const auto t_diff = clock_ptr_->now().seconds() - last_no_critical_dpt_time_;
  return t_diff >= node_param_.on_time_buffer_s.critical_departure;
}

bool BoundaryDeparturePreventionModule::is_critical_departure_persist()
{
  const auto is_critical_departure_found =
    std::any_of(
      g_side_keys.begin(), g_side_keys.end(),
      [&](const auto side_key) {
        const auto & closest_projections = output_.closest_projections_to_bound[side_key];
        return std::any_of(
          closest_projections.rbegin(), closest_projections.rend(),
          [](const auto & pt) { return pt.departure_type == DepartureType::CRITICAL_DEPARTURE; });
      }) &&
    !output_.critical_departure_points.empty();

  if (is_critical_departure_found) {
    last_found_critical_dpt_time_ = clock_ptr_->now().seconds();
    return true;
  }

  const auto t_diff = clock_ptr_->now().seconds() - last_found_critical_dpt_time_;
  return t_diff >= node_param_.off_time_buffer_s.critical_departure;
}

void BoundaryDeparturePreventionModule::publish_visualization_markers()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  slow_down_wall_marker_.markers.clear();

  if (!clock_ptr_) {
    return;
  }
  const auto current_time = clock_ptr_->now();

  publish_virtual_walls(current_time);
  publish_debug_markers(current_time);
}

void BoundaryDeparturePreventionModule::publish_virtual_walls(const rclcpp::Time & current_time)
{
  if (!virtual_wall_publisher_) {
    return;
  }

  for (const auto & [idx, slowdown_interval] :
       output_.slowdown_intervals | ranges::views::enumerate) {
    const auto & [start_pose, end_pose, vel] = slowdown_interval;
    const auto markers_start = autoware::motion_utils::createSlowDownVirtualWallMarker(
      start_pose, "boundary_departure_slow", current_time, static_cast<int32_t>(idx), 0.0, "");
    autoware_utils::append_marker_array(markers_start, &slow_down_wall_marker_);
  }

  for (const auto & [idx, critical_pt] :
       output_.critical_departure_points | ranges::views::enumerate) {
    const auto markers_end = autoware::motion_utils::createStopVirtualWallMarker(
      critical_pt.pose_on_current_ref_traj, "boundary_departure_critical", current_time,
      static_cast<int32_t>(idx + output_.departure_intervals.size() + 1), 0.0);
    autoware_utils::append_marker_array(markers_end, &slow_down_wall_marker_);
  }

  virtual_wall_publisher_->publish(slow_down_wall_marker_);
}

void BoundaryDeparturePreventionModule::publish_debug_markers(const rclcpp::Time & current_time)
{
  debug_marker_.markers.clear();

  if (!debug_publisher_ || !ego_pred_traj_ptr_ || ego_pred_traj_ptr_->points.empty()) {
    return;
  }
  const auto ego_z_position = ego_pred_traj_ptr_->points.front().pose.position.z;
  autoware_utils::append_marker_array(
    debug::create_debug_marker_array(
      output_, *ego_pred_traj_ptr_, current_time, ego_z_position, node_param_),
    &debug_marker_);

  debug_publisher_->publish(debug_marker_);
}
}  // namespace autoware::motion_velocity_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::experimental::BoundaryDeparturePreventionModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
