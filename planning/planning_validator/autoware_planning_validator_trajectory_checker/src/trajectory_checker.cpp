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

#include "autoware/planning_validator_trajectory_checker/trajectory_checker.hpp"

#include "autoware/planning_validator_trajectory_checker/utils.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <angles/angles/angles.h>

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using autoware_utils::get_or_declare_parameter;

void TrajectoryChecker::init(
  rclcpp::Node & node, const std::string & name,
  const std::shared_ptr<PlanningValidatorContext> & context)
{
  module_name_ = name;

  clock_ = node.get_clock();
  logger_ = node.get_logger();

  context_ = context;

  setup_parameters(node);

  setup_diag();
}

void TrajectoryChecker::setup_parameters(rclcpp::Node & node)
{
  const int default_handling_value = get_or_declare_parameter<int>(node, "default_handling_type");

  auto set_common_params = [&](auto & param, const std::string & key) {
    param.enable = get_or_declare_parameter<bool>(node, key + ".enable");
    const auto value = node.declare_parameter<int>(key + ".handling_type", default_handling_value);
    param.handling_type = get_handling_type(value);
    param.override_error_diag = node.declare_parameter<bool>(key + ".override_error_diag", false);
  };

  auto set_validation_params = [&](auto & param, const std::string & key) {
    param.threshold = get_or_declare_parameter<double>(node, key + ".threshold");
    set_common_params(param, key);
  };

  auto & p = params_;
  const std::string t = "trajectory_checker.";
  set_validation_params(p.interval, t + "interval");
  set_validation_params(p.relative_angle, t + "relative_angle");
  set_validation_params(p.curvature, t + "curvature");
  set_validation_params(p.steering, t + "steering");
  set_validation_params(p.steering_rate, t + "steering_rate");
  set_validation_params(p.lateral_jerk, t + "lateral_jerk");
  set_validation_params(p.lateral_accel, t + "lateral_accel");
  set_validation_params(p.max_lon_accel, t + "max_lon_accel");
  set_validation_params(p.min_lon_accel, t + "min_lon_accel");
  set_validation_params(p.distance_deviation, t + "distance_deviation");
  set_validation_params(p.lon_distance_deviation, t + "lon_distance_deviation");
  set_validation_params(p.velocity_deviation, t + "velocity_deviation");

  set_validation_params(p.yaw_deviation, t + "yaw_deviation");
  p.yaw_deviation.th_trajectory_yaw_shift =
    get_or_declare_parameter<double>(node, t + "yaw_deviation.th_trajectory_yaw_shift");

  set_common_params(p.trajectory_shift, t + "trajectory_shift");
  p.trajectory_shift.lat_shift_th =
    get_or_declare_parameter<double>(node, t + "trajectory_shift.lat_shift_th");
  p.trajectory_shift.forward_shift_th =
    get_or_declare_parameter<double>(node, t + "trajectory_shift.forward_shift_th");
  p.trajectory_shift.backward_shift_th =
    get_or_declare_parameter<double>(node, t + "trajectory_shift.backward_shift_th");

  set_common_params(p.forward_trajectory_length, t + "forward_trajectory_length");
  p.forward_trajectory_length.acceleration =
    get_or_declare_parameter<double>(node, t + "forward_trajectory_length.acceleration");
  p.forward_trajectory_length.margin =
    get_or_declare_parameter<double>(node, t + "forward_trajectory_length.margin");
}

void TrajectoryChecker::setup_diag()
{
  if (!context_->diag_updater) return;

  const auto & status = context_->validation_status;

  std::string ns = "trajectory_validation_";
  const auto add_diag = [&](
                          const std::string & name, const bool & status, const std::string & msg) {
    context_->diag_updater->add(
      ns + name, [&, msg = msg](auto & stat) { set_diag_status(stat, status, msg); });
  };

  add_diag("size", status->is_valid_size, "invalid trajectory size is found");
  add_diag("finite", status->is_valid_finite_value, "infinite value is found");
  add_diag("interval", status->is_valid_interval, "points interval is too large");
  add_diag("relative_angle", status->is_valid_relative_angle, "relative angle is too large");
  add_diag("curvature", status->is_valid_curvature, "curvature is too large");
  add_diag(
    "lateral_acceleration", status->is_valid_lateral_acc, "lateral acceleration is too large");
  add_diag("acceleration", status->is_valid_longitudinal_max_acc, "acceleration is too large");
  add_diag("deceleration", status->is_valid_longitudinal_min_acc, "deceleration is too large");
  add_diag("steering", status->is_valid_steering, "steering angle is too large");
  add_diag("steering_rate", status->is_valid_steering_rate, "steering rate is too large");
  add_diag(
    "velocity_deviation", status->is_valid_velocity_deviation, "velocity deviation is too large");
  add_diag(
    "distance_deviation", status->is_valid_distance_deviation, "distance deviation is too large");
  add_diag(
    "longitudinal_distance_deviation", status->is_valid_longitudinal_distance_deviation,
    "longitudinal distance deviation is too large");
  add_diag(
    "yaw_deviation", status->is_valid_yaw_deviation,
    "difference between vehicle yaw and closest trajectory yaw is too large");
  add_diag(
    "forward_trajectory_length", status->is_valid_forward_trajectory_length,
    "trajectory length is too short");
  add_diag(
    "trajectory_shift", status->is_valid_trajectory_shift, "detected sudden shift in trajectory");
}

void TrajectoryChecker::set_diag_status(
  DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const
{
  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "validated.");
    return;
  }

  const auto invalid_count = context_->validation_status->invalid_count;
  const auto count_threshold = context_->params.diag_error_count_threshold;
  if (invalid_count < count_threshold) {
    const auto warn_msg =
      msg + " (invalid count is less than error threshold: " + std::to_string(invalid_count) +
      " < " + std::to_string(count_threshold) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
    return;
  }

  if (override_all_error_diag_) {
    const auto warn_msg = msg + " (error diag was overriden internally)";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
    return;
  }

  stat.summary(DiagnosticStatus::ERROR, msg);
}

void TrajectoryChecker::validate()
{
  auto & status = context_->validation_status;

  const auto terminateValidation = [&](const auto & ss) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 3000, ss);
  };

  override_all_error_diag_ = false;

  status->is_valid_size = check_valid_size();
  if (!status->is_valid_size) {
    return terminateValidation(
      "trajectory has invalid point size (" + std::to_string(status->trajectory_size) +
      "). Stop validation process, raise an error.");
  }

  status->is_valid_finite_value = check_valid_finite_value();
  if (!status->is_valid_finite_value) {
    return terminateValidation(
      "trajectory has invalid value (NaN, Inf, etc). Stop validation process, raise an error.");
  }

  status->is_valid_interval = check_valid_interval();
  status->is_valid_longitudinal_max_acc = check_valid_max_longitudinal_acceleration();
  status->is_valid_longitudinal_min_acc = check_valid_min_longitudinal_acceleration();
  status->is_valid_velocity_deviation = check_valid_velocity_deviation();
  status->is_valid_distance_deviation = check_valid_distance_deviation();
  status->is_valid_longitudinal_distance_deviation = check_valid_longitudinal_distance_deviation();
  status->is_valid_yaw_deviation = check_valid_yaw_deviation();
  status->is_valid_forward_trajectory_length = check_valid_forward_trajectory_length();
  status->is_valid_trajectory_shift = check_trajectory_shift();
  status->is_valid_relative_angle = check_valid_relative_angle();
  status->is_valid_curvature = check_valid_curvature();
  status->is_valid_lateral_acc = check_valid_lateral_acceleration();
  status->is_valid_lateral_jerk = check_valid_lateral_jerk();
  status->is_valid_steering = check_valid_steering();
  status->is_valid_steering_rate = check_valid_steering_rate();
}

bool TrajectoryChecker::check_valid_size()
{
  auto & status = context_->validation_status;
  status->trajectory_size = context_->data->current_trajectory->points.size();
  return status->trajectory_size >= 2;
}

bool TrajectoryChecker::check_valid_finite_value()
{
  const auto & trajectory = *context_->data->current_trajectory;
  for (const auto & p : trajectory.points) {
    if (!trajectory_checker_utils::checkFinite(p)) return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_interval()
{
  if (!params_.interval.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->current_trajectory;

  const auto [max_interval_distance, i] =
    trajectory_checker_utils::calcMaxIntervalDistance(trajectory);
  status->max_interval_distance = max_interval_distance;

  if (max_interval_distance > params_.interval.threshold) {
    if (i > 0) {
      const auto & p = trajectory.points;
      context_->debug_pose_publisher->pushPoseMarker(p.at(i - 1), "trajectory_interval");
      context_->debug_pose_publisher->pushPoseMarker(p.at(i), "trajectory_interval");
    }
    context_->set_handling(params_.interval.handling_type);
    override_all_error_diag_ |= params_.interval.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_relative_angle()
{
  if (!params_.relative_angle.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->resampled_current_trajectory;

  const auto [max_relative_angle, i] = trajectory_checker_utils::calcMaxRelativeAngles(trajectory);
  status->max_relative_angle = max_relative_angle;

  if (max_relative_angle > params_.relative_angle.threshold) {
    const auto & p = trajectory.points;
    if (i < p.size() - 3) {
      context_->debug_pose_publisher->pushPoseMarker(p.at(i), "trajectory_relative_angle", 0);
      context_->debug_pose_publisher->pushPoseMarker(p.at(i + 1), "trajectory_relative_angle", 1);
      context_->debug_pose_publisher->pushPoseMarker(p.at(i + 2), "trajectory_relative_angle", 2);
    }
    context_->set_handling(params_.relative_angle.handling_type);
    override_all_error_diag_ |= params_.relative_angle.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_curvature()
{
  if (!params_.curvature.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->resampled_current_trajectory;

  const auto [max_curvature, i] = trajectory_checker_utils::calcMaxCurvature(trajectory);
  status->max_curvature = max_curvature;
  if (max_curvature > params_.curvature.threshold) {
    const auto & p = trajectory.points;
    if (i > 0 && i < p.size() - 1) {
      context_->debug_pose_publisher->pushPoseMarker(p.at(i - 1), "trajectory_curvature");
      context_->debug_pose_publisher->pushPoseMarker(p.at(i), "trajectory_curvature");
      context_->debug_pose_publisher->pushPoseMarker(p.at(i + 1), "trajectory_curvature");
    }
    context_->set_handling(params_.curvature.handling_type);
    override_all_error_diag_ |= params_.curvature.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_lateral_acceleration()
{
  if (!params_.lateral_accel.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->resampled_current_trajectory;

  const auto [max_lateral_acc, i] =
    trajectory_checker_utils::calcMaxLateralAcceleration(trajectory);
  status->max_lateral_acc = max_lateral_acc;
  if (max_lateral_acc > params_.lateral_accel.threshold) {
    context_->debug_pose_publisher->pushPoseMarker(trajectory.points.at(i), "lateral_acceleration");
    context_->set_handling(params_.lateral_accel.handling_type);
    override_all_error_diag_ |= params_.lateral_accel.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_lateral_jerk()
{
  if (!params_.lateral_jerk.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->resampled_current_trajectory;

  const auto [max_lateral_jerk, i] = trajectory_checker_utils::calc_max_lateral_jerk(trajectory);
  status->max_lateral_jerk = max_lateral_jerk;
  if (max_lateral_jerk > params_.lateral_jerk.threshold) {
    context_->debug_pose_publisher->pushPoseMarker(trajectory.points.at(i), "lateral_jerk");
    context_->set_handling(params_.lateral_jerk.handling_type);
    override_all_error_diag_ |= params_.lateral_jerk.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_min_longitudinal_acceleration()
{
  if (!params_.min_lon_accel.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->current_trajectory;

  const auto [min_longitudinal_acc, i] =
    trajectory_checker_utils::getMinLongitudinalAcc(trajectory);
  status->min_longitudinal_acc = min_longitudinal_acc;

  if (min_longitudinal_acc < params_.min_lon_accel.threshold) {
    context_->debug_pose_publisher->pushPoseMarker(
      trajectory.points.at(i).pose, "min_longitudinal_acc");
    context_->set_handling(params_.min_lon_accel.handling_type);
    override_all_error_diag_ |= params_.min_lon_accel.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_max_longitudinal_acceleration()
{
  if (!params_.max_lon_accel.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->current_trajectory;

  const auto [max_longitudinal_acc, i] =
    trajectory_checker_utils::getMaxLongitudinalAcc(trajectory);
  status->max_longitudinal_acc = max_longitudinal_acc;

  if (max_longitudinal_acc > params_.max_lon_accel.threshold) {
    context_->debug_pose_publisher->pushPoseMarker(
      trajectory.points.at(i).pose, "max_longitudinal_acc");
    context_->set_handling(params_.max_lon_accel.handling_type);
    override_all_error_diag_ |= params_.max_lon_accel.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_steering()
{
  if (!params_.steering.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->resampled_current_trajectory;
  const auto vehicle_wheel_base_m = context_->vehicle_info.wheel_base_m;

  const auto [max_steering, i] =
    trajectory_checker_utils::calcMaxSteeringAngles(trajectory, vehicle_wheel_base_m);
  status->max_steering = max_steering;

  if (max_steering > params_.steering.threshold) {
    context_->debug_pose_publisher->pushPoseMarker(trajectory.points.at(i).pose, "max_steering");
    context_->set_handling(params_.steering.handling_type);
    override_all_error_diag_ |= params_.steering.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_steering_rate()
{
  if (!params_.steering.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->resampled_current_trajectory;
  const auto vehicle_wheel_base_m = context_->vehicle_info.wheel_base_m;

  const auto [max_steering_rate, i] =
    trajectory_checker_utils::calcMaxSteeringRates(trajectory, vehicle_wheel_base_m);
  status->max_steering_rate = max_steering_rate;

  if (max_steering_rate > params_.steering_rate.threshold) {
    context_->debug_pose_publisher->pushPoseMarker(
      trajectory.points.at(i).pose, "max_steering_rate");
    context_->set_handling(params_.steering_rate.handling_type);
    override_all_error_diag_ |= params_.steering_rate.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_velocity_deviation()
{
  if (!params_.velocity_deviation.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;

  if (!data->nearest_point_index) return false;
  const auto idx = *data->nearest_point_index;

  const auto & trajectory = *data->current_trajectory;
  const auto ego_speed = data->current_kinematics->twist.twist.linear.x;

  status->velocity_deviation =
    std::abs(trajectory.points.at(idx).longitudinal_velocity_mps - ego_speed);

  if (status->velocity_deviation > params_.velocity_deviation.threshold) {
    context_->set_handling(params_.velocity_deviation.handling_type);
    override_all_error_diag_ |= params_.velocity_deviation.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_distance_deviation()
{
  if (!params_.distance_deviation.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;

  if (!data->nearest_point_index) return false;
  const auto idx = *data->nearest_segment_index;

  const auto & trajectory = data->current_trajectory->points;
  const auto & ego_position = data->current_kinematics->pose.pose.position;
  status->distance_deviation = motion_utils::calcLateralOffset(trajectory, ego_position, idx);

  if (std::abs(status->distance_deviation) > params_.distance_deviation.threshold) {
    context_->set_handling(params_.distance_deviation.handling_type);
    override_all_error_diag_ |= params_.distance_deviation.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_longitudinal_distance_deviation()
{
  if (!params_.lon_distance_deviation.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->current_trajectory;

  if (trajectory.points.size() < 2) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 3000, "Trajectory size is invalid to calculate distance deviation.");
    return false;
  }

  if (!data->nearest_point_index) return false;
  const auto idx = *data->nearest_point_index;

  const auto & ego_pose = data->current_kinematics->pose.pose;

  if (0 < idx && idx < trajectory.points.size() - 1) {
    return true;  // ego-nearest point exists between trajectory points.
  }

  // Check if the valid longitudinal deviation for given segment index
  const auto has_valid_lon_deviation = [&](const size_t seg_idx, const bool is_last) {
    auto long_offset = autoware::motion_utils::calcLongitudinalOffsetToSegment(
      trajectory.points, seg_idx, ego_pose.position);

    // for last, need to remove distance for the last segment.
    if (is_last) {
      const auto size = trajectory.points.size();
      long_offset -= autoware_utils::calc_distance2d(
        trajectory.points.at(size - 1), trajectory.points.at(size - 2));
    }

    status->longitudinal_distance_deviation = long_offset;
    return std::abs(status->longitudinal_distance_deviation) <
           params_.lon_distance_deviation.threshold;
  };

  // Make sure the trajectory is far AHEAD from ego.
  if (idx == 0) {
    const auto seg_idx = 0;
    if (!has_valid_lon_deviation(seg_idx, false)) {
      context_->set_handling(params_.lon_distance_deviation.handling_type);
      override_all_error_diag_ |= params_.lon_distance_deviation.override_error_diag;
      return false;
    }
    return true;
  }

  // Make sure the trajectory is far BEHIND from ego.
  if (idx == trajectory.points.size() - 1) {
    const auto seg_idx = trajectory.points.size() - 2;
    if (!has_valid_lon_deviation(seg_idx, true)) {
      context_->set_handling(params_.lon_distance_deviation.handling_type);
      override_all_error_diag_ |= params_.lon_distance_deviation.override_error_diag;
      return false;
    }
    return true;
  }

  return true;
}

double nearest_trajectory_yaw_shift(
  const Trajectory & last_valid_trajectory,
  const autoware_planning_msgs::msg::TrajectoryPoint & trajectory_point)
{
  const auto interpolated_previous_trajectory_point =
    motion_utils::calcInterpolatedPoint(last_valid_trajectory, trajectory_point.pose);
  const auto yaw_shift_with_previous_trajectory = std::abs(
    angles::shortest_angular_distance(
      tf2::getYaw(trajectory_point.pose.orientation),
      tf2::getYaw(interpolated_previous_trajectory_point.pose.orientation)));
  return yaw_shift_with_previous_trajectory;
}

bool TrajectoryChecker::check_valid_yaw_deviation()
{
  if (!params_.yaw_deviation.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;
  const auto & trajectory = *data->current_trajectory;
  const auto & ego_pose = data->current_kinematics->pose.pose;

  const auto interpolated_trajectory_point =
    motion_utils::calcInterpolatedPoint(trajectory, ego_pose);
  status->yaw_deviation = std::abs(
    angles::shortest_angular_distance(
      tf2::getYaw(interpolated_trajectory_point.pose.orientation),
      tf2::getYaw(ego_pose.orientation)));

  const auto check_condition =
    !data->last_valid_trajectory ||
    nearest_trajectory_yaw_shift(*data->last_valid_trajectory, interpolated_trajectory_point) >
      params_.yaw_deviation.th_trajectory_yaw_shift;
  if (check_condition && status->yaw_deviation > params_.yaw_deviation.threshold) {
    context_->set_handling(params_.yaw_deviation.handling_type);
    override_all_error_diag_ |= params_.yaw_deviation.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_valid_forward_trajectory_length()
{
  if (!params_.forward_trajectory_length.enable) {
    return true;
  }

  const auto & data = context_->data;
  auto & status = context_->validation_status;

  const auto & trajectory = *data->current_trajectory;
  const auto & ego_pose = data->current_kinematics->pose.pose;

  const auto ego_speed = std::abs(data->current_kinematics->twist.twist.linear.x);
  if (ego_speed < 1.0 / 3.6) {
    return true;  // Ego is almost stopped.
  }

  const auto forward_length = autoware::motion_utils::calcSignedArcLength(
    trajectory.points, ego_pose.position, trajectory.points.size() - 1);

  const auto acc = params_.forward_trajectory_length.acceleration;
  const auto forward_length_required =
    ego_speed * ego_speed / (2.0 * std::abs(acc)) - params_.forward_trajectory_length.margin;

  status->forward_trajectory_length_required = forward_length_required;
  status->forward_trajectory_length_measured = forward_length;

  if (forward_length < forward_length_required) {
    context_->set_handling(params_.forward_trajectory_length.handling_type);
    override_all_error_diag_ |= params_.forward_trajectory_length.override_error_diag;
    return false;
  }
  return true;
}

bool TrajectoryChecker::check_trajectory_shift()
{
  bool is_valid = true;
  const auto & data = context_->data;
  if (!params_.trajectory_shift.enable || !data->last_valid_trajectory) {
    return is_valid;
  }

  auto & status = context_->validation_status;
  const auto & trajectory = *data->current_trajectory;
  const auto & prev_trajectory = *data->last_valid_trajectory;
  const auto & ego_pose = data->current_kinematics->pose.pose;

  const auto nearest_seg_idx = data->nearest_segment_index;
  const auto prev_nearest_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(prev_trajectory.points, ego_pose);

  if (!nearest_seg_idx || !prev_nearest_seg_idx) {
    return is_valid;
  }

  const auto & nearest_pose = trajectory.points.at(*nearest_seg_idx).pose;
  const auto & prev_nearest_pose = prev_trajectory.points.at(*prev_nearest_seg_idx).pose;

  const auto & ego_lat_dist =
    std::abs(autoware_utils::calc_lateral_deviation(ego_pose, nearest_pose.position));

  const auto lat_shift =
    std::abs(autoware_utils::calc_lateral_deviation(prev_nearest_pose, nearest_pose.position));

  static constexpr auto epsilon = 0.01;
  status->lateral_shift = lat_shift > epsilon ? lat_shift : 0.0;

  if (
    ego_lat_dist > params_.trajectory_shift.lat_shift_th &&
    lat_shift > params_.trajectory_shift.lat_shift_th) {
    context_->set_handling(params_.trajectory_shift.handling_type);
    override_all_error_diag_ |= params_.trajectory_shift.override_error_diag;
    context_->debug_pose_publisher->pushPoseMarker(nearest_pose, "trajectory_shift");
    is_valid = false;
  }

  const auto is_check_lon_shift = std::invoke([&]() {
    if (*prev_nearest_seg_idx == prev_trajectory.points.size() - 2) {
      return false;  // no need to check longitudinal shift if at the end of previous trajectory
    }
    if (*nearest_seg_idx > 0 && *nearest_seg_idx < trajectory.points.size() - 2) {
      return false;  // no need to check longitudinal shift if ego is within the current trajectory
    }
    return true;
  });

  // if nearest segment is within the trajectory no need to check longitudinal shift
  if (!is_check_lon_shift) {
    status->longitudinal_shift = 0.0;
    return is_valid;
  }

  const auto lon_shift =
    autoware_utils::calc_longitudinal_deviation(prev_nearest_pose, nearest_pose.position);

  status->longitudinal_shift = std::abs(lon_shift) > epsilon ? lon_shift : 0.0;

  // if the nearest segment is the first segment, check forward shift
  if (*nearest_seg_idx == 0) {
    if (lon_shift > params_.trajectory_shift.forward_shift_th) {
      context_->set_handling(params_.trajectory_shift.handling_type);
      override_all_error_diag_ |= params_.trajectory_shift.override_error_diag;
      context_->debug_pose_publisher->pushPoseMarker(nearest_pose, "trajectory_shift");
      is_valid = false;
    }
    return is_valid;
  }

  // if the nearest segment is the last segment, check backward shift
  if (lon_shift < 0.0 && std::abs(lon_shift) > params_.trajectory_shift.backward_shift_th) {
    context_->set_handling(params_.trajectory_shift.handling_type);
    override_all_error_diag_ |= params_.trajectory_shift.override_error_diag;
    context_->debug_pose_publisher->pushPoseMarker(nearest_pose, "trajectory_shift");
    is_valid = false;
  }

  return is_valid;
}

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::TrajectoryChecker, autoware::planning_validator::PluginInterface)
