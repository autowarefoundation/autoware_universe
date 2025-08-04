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

#include <autoware/boundary_departure_checker/utils.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <magic_enum.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/view.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
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
      const auto type = std::invoke([&]() {
        if (output_.diagnostic_output[DepartureType::CRITICAL_DEPARTURE]) {
          return DepartureType::CRITICAL_DEPARTURE;
        }
        if (output_.diagnostic_output[DepartureType::APPROACHING_DEPARTURE]) {
          return DepartureType::APPROACHING_DEPARTURE;
        }
        if (output_.diagnostic_output[DepartureType::NEAR_BOUNDARY]) {
          return DepartureType::NEAR_BOUNDARY;
        }
        return DepartureType::NONE;
      });

      auto lvl = node_param_.diagnostic_level[type];
      auto msg = to_string(type);

      if (lvl != DiagStatus::OK && type != DepartureType::NONE) {
        RCLCPP_ERROR_THROTTLE(logger_, *clock_ptr_, 1000, "%s", msg.c_str());
      }

      stat.summary(lvl, msg);
    });
  last_found_time_ptr_ = std::make_unique<double>(clock_ptr_->now().seconds());
  last_lost_time_ptr_ = std::make_unique<double>(clock_ptr_->now().seconds());
}

void BoundaryDeparturePreventionModule::update_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  using autoware_utils::update_param;
  auto & pp = node_param_;

  const std::string module_name{"boundary_departure_prevention."};
  update_param(
    parameters, module_name + "boundary_types_to_detect", pp.bdc_param.boundary_types_to_detect);
  update_param(
    parameters, module_name + "th_max_lateral_query_num", pp.bdc_param.th_max_lateral_query_num);

  [[maybe_unused]] const auto abnormality_params = [&module_name, &parameters, &pp]() {
    const std::string ns_abnormality{module_name + "abnormality."};
    const std::string ns_normal_abnormality{ns_abnormality + "normal."};
    const std::string ns_steering_abnormality{ns_abnormality + "steering."};
    const std::string ns_localization_abnormality{ns_abnormality + "localization."};
    const std::string ns_longitudinal_abnormality{ns_abnormality + "longitudinal."};

    const auto has_type = [&](const AbnormalityType type_to_check) {
      return std::any_of(
        pp.bdc_param.abnormality_types_to_compensate.begin(),
        pp.bdc_param.abnormality_types_to_compensate.end(),
        [&](const AbnormalityType type) { return type == type_to_check; });
    };

    std::vector<AbnormalityType> abnormality_types_to_compensate;
    AbnormalitiesConfigs configs;

    auto compensate_normal = has_type(AbnormalityType::NORMAL);
    update_param(parameters, ns_normal_abnormality + "enable", compensate_normal);
    if (compensate_normal) {
      abnormality_types_to_compensate.emplace_back(AbnormalityType::NORMAL);

      NormalConfig normal_config;
      const std::string footprint_envelop_ns{ns_normal_abnormality + "footprint_envelop."};

      update_param(
        parameters, footprint_envelop_ns + "lat_m", normal_config.footprint_envelop.lat_m);
      update_param(
        parameters, footprint_envelop_ns + "lon_m", normal_config.footprint_envelop.lon_m);
      configs.insert({AbnormalityType::NORMAL, normal_config});
    }

    auto compensate_steering = has_type(AbnormalityType::STEERING);
    update_param(parameters, ns_steering_abnormality + "enable", compensate_steering);
    if (compensate_steering) {
      SteeringConfig steering_config;
      abnormality_types_to_compensate.emplace_back(AbnormalityType::STEERING);
      update_param(
        parameters, ns_steering_abnormality + "steering_rate_rps",
        steering_config.steering_rate_rps);
      configs.insert({AbnormalityType::STEERING, steering_config});
    }

    auto compensate_localization = has_type(AbnormalityType::LOCALIZATION);
    update_param(parameters, ns_localization_abnormality + "enable", compensate_localization);

    if (compensate_localization) {
      abnormality_types_to_compensate.emplace_back(AbnormalityType::LOCALIZATION);
      LocalizationConfig localization_config;
      const std::string footprint_envelop_ns{ns_localization_abnormality + "footprint_envelop."};
      update_param(
        parameters, footprint_envelop_ns + "lat_m", localization_config.footprint_envelop.lat_m);
      update_param(
        parameters, footprint_envelop_ns + "lon_m", localization_config.footprint_envelop.lon_m);
      configs.insert({AbnormalityType::LOCALIZATION, localization_config});
    }

    auto compensate_longitudinal = has_type(AbnormalityType::LONGITUDINAL);
    update_param(parameters, ns_longitudinal_abnormality + "enable", compensate_longitudinal);
    if (compensate_longitudinal) {
      abnormality_types_to_compensate.emplace_back(AbnormalityType::LONGITUDINAL);
      LongitudinalConfig longitudinal_config;
      const std::string lon_tracking_ns{ns_longitudinal_abnormality + "lon_tracking."};

      update_param(parameters, lon_tracking_ns + "scale", longitudinal_config.lon_tracking.scale);

      update_param(
        parameters, lon_tracking_ns + "extra_margin_m",
        longitudinal_config.lon_tracking.extra_margin_m);
      configs.insert({AbnormalityType::LONGITUDINAL, longitudinal_config});
    }

    pp.bdc_param.abnormality_types_to_compensate = abnormality_types_to_compensate;
    pp.bdc_param.abnormality_configs = configs;
  };
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
    create_subscription(&node, "/planning/mission_planning/route");
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

  if (const auto invalid_data_opt = is_data_invalid(raw_trajectory_points)) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "%s", invalid_data_opt->c_str());
    return {};
  }

  if (const auto is_timeout_opt = is_data_timeout(planner_data->current_odometry)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_ptr_, throttle_duration_ms, "%s", is_timeout_opt->c_str());
    return {};
  }

  if (const auto is_new_route = is_route_changed()) {
    RCLCPP_WARN(logger_, "%s. Reset output.", is_new_route->c_str());
    output_ = Output();
  }

  if (!is_autonomous_mode()) {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_ptr_, throttle_duration_ms, "Not in autonomous mode.");
    updater_ptr_->force_update();
    return {};
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

    processing_time_publisher_->publish(std::invoke([&]() {
      autoware_internal_debug_msgs::msg::Float64Stamped msg;
      msg.stamp = clock_ptr_->now();
      msg.data = stopwatch_ms.toc();
      return msg;
    }));

    updater_ptr_->force_update();
    if (!result_opt) {
      RCLCPP_DEBUG(logger_, "Planning skipped: %s", result_opt.error().c_str());
      return {};
    }

    return *result_opt;
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Exception is caught: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger_, "Unknown exception is caught.");
  }

  return {};
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
      *ref_traj_pts_opt, output_.abnormalities_data.projections_to_bound, curr_vel, curr_acc);
  toc_curr_watch("get_ref_traj");

  if (!closest_projections_to_bound_opt) {
    return tl::make_unexpected(closest_projections_to_bound_opt.error());
  }

  output_.closest_projections_to_bound = *closest_projections_to_bound_opt;

  const auto ego_dist_on_traj_m =
    experimental::trajectory::closest(*ref_traj_pts_opt, curr_pose.pose);
  toc_curr_watch("chk_ego_dist_on_traj");

  const auto lon_offset_m = [&vehicle_info](const bool take_front_offset) {
    return take_front_offset ? vehicle_info.max_longitudinal_offset_m
                             : vehicle_info.min_longitudinal_offset_m;
  };
  const auto ego_dist_on_traj_with_offset_m = [&](const bool take_front_offset) {
    return ego_dist_on_traj_m + lon_offset_m(take_front_offset);
  };

  output_.departure_points = boundary_departure_checker_ptr_->get_departure_points(
    output_.closest_projections_to_bound, lon_offset_m(planner_data->is_driving_forward));
  toc_curr_watch("get_departure_points");

  utils::update_critical_departure_points(
    output_.departure_points, output_.critical_departure_points, *ref_traj_pts_opt,
    node_param_.bdc_param.th_point_merge_distance_m,
    ego_dist_on_traj_with_offset_m(!planner_data->is_driving_forward),
    node_param_.th_pt_shift_dist_m, node_param_.th_pt_shift_angle_rad);
  toc_curr_watch("update_critical_departure_points");

  const auto is_departure_persist = std::invoke([&]() {
    const auto is_found =
      std::any_of(g_side_keys.begin(), g_side_keys.end(), [&](const auto side_key) {
        return !closest_projections_to_bound_opt.value()[side_key].empty();
      });

    if (!is_found) {
      *last_lost_time_ptr_ = clock_ptr_->now().seconds();
      return false;
    }

    const auto t_diff = clock_ptr_->now().seconds() - *last_lost_time_ptr_;
    return t_diff >= node_param_.on_time_buffer_s;
  });

  if (output_.departure_intervals.empty() && is_departure_persist) {
    output_.departure_intervals = utils::init_departure_intervals(
      *ref_traj_pts_opt, output_.departure_points,
      ego_dist_on_traj_with_offset_m(!planner_data->is_driving_forward),
      node_param_.slow_down_types);
  }

  if (!output_.departure_intervals.empty()) {
    auto & departure_intervals_mut = output_.departure_intervals;
    const auto & ref_traj_front_pt = raw_trajectory_points.front();

    const auto is_reset_interval = std::invoke([&]() {
      const auto is_departure_found = std::any_of(
        g_side_keys.begin(), g_side_keys.end(),
        [&](const auto side_key) { return !output_.departure_points[side_key].empty(); });

      if (is_departure_found) {
        *last_found_time_ptr_ = clock_ptr_->now().seconds();
        return false;
      }
      const auto t_diff = clock_ptr_->now().seconds() - *last_found_time_ptr_;
      return t_diff >= node_param_.off_time_buffer_s;
    });

    utils::update_departure_intervals(
      departure_intervals_mut, output_.departure_points, *ref_traj_pts_opt,
      vehicle_info.vehicle_length_m, ref_traj_front_pt,
      ego_dist_on_traj_with_offset_m(!planner_data->is_driving_forward),
      node_param_.th_pt_shift_dist_m, node_param_.th_pt_shift_angle_rad,
      node_param_.slow_down_types, is_reset_interval, is_departure_persist);

    if (is_reset_interval) {
      *last_found_time_ptr_ = clock_ptr_->now().seconds();
    }
    const auto reset_lost_time =
      std::any_of(g_side_keys.begin(), g_side_keys.end(), [&](const auto side_key) {
        return output_.departure_intervals.empty() || output_.departure_points[side_key].empty();
      });

    if (reset_lost_time) {
      *last_lost_time_ptr_ = clock_ptr_->now().seconds();
    }
  }

  toc_curr_watch("update_departure_intervals");

  slow_down_wall_marker_.markers.clear();

  output_.slowdown_intervals = utils::get_slow_down_intervals(
    *ref_traj_pts_opt, output_.departure_intervals, *slow_down_interpolator_ptr_,
    curr_odom.twist.twist.linear.x, planner_data->current_acceleration.accel.accel.linear.x,
    ego_dist_on_traj_with_offset_m(planner_data->is_driving_forward));
  toc_curr_watch("get_slow_down_interval");

  std::vector<SlowdownInterval> slowdown_intervals;
  for (auto && [idx, slowdown_interval] : output_.slowdown_intervals | ranges::views::enumerate) {
    const auto & [start_pose, end_pose, vel] = slowdown_interval;
    slowdown_intervals.emplace_back(start_pose.position, end_pose.position, vel);
    const auto markers_start = autoware::motion_utils::createSlowDownVirtualWallMarker(
      start_pose, "boundary_departure_prevention_start", clock_ptr_->now(),
      static_cast<int32_t>(idx), 0.0, "", planner_data->is_driving_forward);
    autoware_utils::append_marker_array(markers_start, &slow_down_wall_marker_);
    const auto markers_end = autoware::motion_utils::createSlowDownVirtualWallMarker(
      end_pose, "boundary_departure_prevention_end", clock_ptr_->now(),
      static_cast<int32_t>(idx + output_.departure_intervals.size() + 1), 0.0, "",
      planner_data->is_driving_forward);
    autoware_utils::append_marker_array(markers_end, &slow_down_wall_marker_);
  }

  for (auto && [idx, critical_pt] : output_.critical_departure_points | ranges::views::enumerate) {
    const auto markers_end = autoware::motion_utils::createStopVirtualWallMarker(
      critical_pt.point_on_prev_traj.pose, "boundary_departure_prevention_end", clock_ptr_->now(),
      static_cast<int32_t>(idx + output_.departure_intervals.size() + 1), 0.0, "",
      planner_data->is_driving_forward);
    autoware_utils::append_marker_array(markers_end, &slow_down_wall_marker_);
  }
  toc_curr_watch("create_slow_down_interval_marker");

  if (virtual_wall_publisher_) {
    virtual_wall_publisher_->publish(slow_down_wall_marker_);
  }

  if (debug_publisher_) {
    debug_marker_.markers.clear();
    autoware_utils::append_marker_array(
      debug::create_debug_marker_array(
        output_, *ego_pred_traj_ptr_, clock_ptr_, curr_pose.pose.position.z, node_param_),
      &debug_marker_);
    debug_publisher_->publish(debug_marker_);
  }

  toc_curr_watch("check_stopping_dist");

  output_.diagnostic_output = get_diagnostics(
    curr_odom.twist.twist.linear.x,
    ego_dist_on_traj_with_offset_m(planner_data->is_driving_forward));
  toc_curr_watch("get_diagnostics");

  VelocityPlanningResult result;
  result.slowdown_intervals = slowdown_intervals;
  return result;
}

std::unordered_map<DepartureType, bool> BoundaryDeparturePreventionModule::get_diagnostics(
  const double curr_vel, const double dist_with_offset_m)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::unordered_map<DepartureType, bool> diag{
    {DepartureType::NEAR_BOUNDARY, false},
    {DepartureType::APPROACHING_DEPARTURE, false},
    {DepartureType::CRITICAL_DEPARTURE, false}};

  const auto & th_trigger = node_param_.bdc_param.th_trigger;

  const auto has_type = [&](const DepartureType type, const SideKey side_key) {
    const auto is_type = [type](const DeparturePoint & pt) { return pt.departure_type == type; };

    return std::any_of(
      output_.departure_points[side_key].cbegin(), output_.departure_points[side_key].cend(),
      is_type);
  };

  diag[DepartureType::NEAR_BOUNDARY] = std::any_of(
    g_side_keys.begin(), g_side_keys.end(),
    [&](const SideKey side_key) { return has_type(DepartureType::NEAR_BOUNDARY, side_key); });

  diag[DepartureType::APPROACHING_DEPARTURE] =
    std::any_of(g_side_keys.begin(), g_side_keys.end(), [&](const SideKey side_key) {
      return has_type(DepartureType::APPROACHING_DEPARTURE, side_key);
    });

  diag[DepartureType::CRITICAL_DEPARTURE] = std::any_of(
    output_.critical_departure_points.cbegin(), output_.critical_departure_points.cend(),
    [&th_trigger, &curr_vel, &dist_with_offset_m](const DeparturePoint & pt) {
      const auto braking_start_vel =
        std::clamp(curr_vel, th_trigger.th_vel_mps.min, th_trigger.th_vel_mps.max);
      const auto braking_dist = boundary_departure_checker::utils::compute_braking_distance(
        braking_start_vel, 0.0, th_trigger.th_acc_mps2.min, th_trigger.th_jerk_mps3.max,
        th_trigger.brake_delay_s);
      return pt.dist_on_traj - dist_with_offset_m <= braking_dist;
    });

  return diag;
}
}  // namespace autoware::motion_velocity_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::experimental::BoundaryDeparturePreventionModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
