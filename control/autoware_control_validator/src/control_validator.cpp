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

#include "autoware/control_validator/control_validator.hpp"

#include "autoware/control_validator/utils.hpp"
#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace autoware::control_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;

void AccelerationValidator::validate(
  ControlValidatorStatus & res, const Odometry & kinematic_state, const Control & control_cmd,
  const AccelWithCovarianceStamped & loc_acc)
{
  desired_acc_lpf.filter(
    control_cmd.longitudinal.acceleration +
    9.8 * autoware_utils::get_rpy(kinematic_state.pose.pose).y);
  measured_acc_lpf.filter(loc_acc.accel.accel.linear.x);
  if (std::abs(kinematic_state.twist.twist.linear.x) < 0.3) {
    desired_acc_lpf.reset(0.0);
    measured_acc_lpf.reset(0.0);
  }

  res.desired_acc = desired_acc_lpf.getValue().value();
  res.measured_acc = measured_acc_lpf.getValue().value();
  res.is_valid_acc = is_in_error_range();
}

bool AccelerationValidator::is_in_error_range() const
{
  const double des = desired_acc_lpf.getValue().value();
  const double mes = measured_acc_lpf.getValue().value();

  return mes <= des + std::abs(e_scale * des) + e_offset &&
         mes >= des - std::abs(e_scale * des) - e_offset;
}

ControlValidator::ControlValidator(const rclcpp::NodeOptions & options)
: Node("control_validator", options), validation_params_(), vehicle_info_()
{
  using std::placeholders::_1;

  sub_control_cmd_ = create_subscription<Control>(
    "~/input/control_cmd", 1, std::bind(&ControlValidator::on_control_cmd, this, _1));
  sub_kinematics_ =
    autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>::create_subscription(
      this, "~/input/kinematics", 1);
  sub_reference_traj_ =
    autoware_utils::InterProcessPollingSubscriber<Trajectory>::create_subscription(
      this, "~/input/reference_trajectory", 1);
  sub_predicted_traj_ =
    autoware_utils::InterProcessPollingSubscriber<Trajectory>::create_subscription(
      this, "~/input/predicted_trajectory", 1);
  sub_measured_acc_ =
    autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>::create_subscription(
      this, "~/input/measured_acceleration", 1);

  pub_status_ = create_publisher<ControlValidatorStatus>("~/output/validation_status", 1);

  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);

  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);

  debug_pose_publisher_ = std::make_shared<ControlValidatorDebugMarkerPublisher>(this);

  setup_parameters();

  setup_diag();
}

void ControlValidator::setup_parameters()
{
  diag_error_count_threshold_ = declare_parameter<int64_t>("diag_error_count_threshold");
  display_on_terminal_ = declare_parameter<bool>("display_on_terminal");

  {
    auto & p = validation_params_;
    const std::string t = "thresholds.";
    p.max_distance_deviation_threshold = declare_parameter<double>(t + "max_distance_deviation");
    p.rolling_back_velocity = declare_parameter<double>(t + "rolling_back_velocity");
    p.over_velocity_offset = declare_parameter<double>(t + "over_velocity_offset");
    p.over_velocity_ratio = declare_parameter<double>(t + "over_velocity_ratio");
    p.overrun_stop_point_dist = declare_parameter<double>(t + "overrun_stop_point_dist");
    p.nominal_latency_threshold = declare_parameter<double>(t + "nominal_latency");
  }

  const auto vel_lpf_gain = declare_parameter<double>("vel_lpf_gain");
  vehicle_vel_.setGain(vel_lpf_gain);
  target_vel_.setGain(vel_lpf_gain);

  hold_velocity_error_until_stop_ = declare_parameter<bool>("hold_velocity_error_until_stop");

  try {
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  } catch (...) {
    vehicle_info_.front_overhang_m = 0.5;
    vehicle_info_.wheel_base_m = 4.0;
    RCLCPP_ERROR(
      get_logger(),
      "failed to get vehicle info. use default value. vehicle_info_.front_overhang_m: %.2f, "
      "vehicle_info_.wheel_base_m: %.2f",
      vehicle_info_.front_overhang_m, vehicle_info_.wheel_base_m);
  }
}

void ControlValidator::set_status(
  DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const
{
  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "validated.");
  } else if (validation_status_.invalid_count < diag_error_count_threshold_) {
    const auto warn_msg = msg + " (invalid count is less than error threshold: " +
                          std::to_string(validation_status_.invalid_count) + " < " +
                          std::to_string(diag_error_count_threshold_) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
  } else {
    stat.summary(DiagnosticStatus::ERROR, msg);
  }
}

void ControlValidator::setup_diag()
{
  auto & d = diag_updater_;
  d.setHardwareID("control_validator");

  std::string ns = "control_validation_";
  d.add(ns + "max_distance_deviation", [&](auto & stat) {
    set_status(
      stat, validation_status_.is_valid_max_distance_deviation,
      "control output is deviated from trajectory");
  });
  d.add(ns + "acceleration_error", [&](auto & stat) {
    set_status(
      stat, validation_status_.is_valid_acc,
      "Measured acceleration and desired acceleration is deviated");
  });
  d.add(ns + "rolling_back", [&](auto & stat) {
    set_status(
      stat, !validation_status_.is_rolling_back,
      "The vehicle is rolling back. The velocity has the opposite sign to the target.");
  });
  d.add(ns + "over_velocity", [&](auto & stat) {
    set_status(
      stat, !validation_status_.is_over_velocity,
      "The vehicle is over-speeding against the target.");
  });
  d.add(ns + "overrun_stop_point", [&](auto & stat) {
    set_status(
      stat, !validation_status_.has_overrun_stop_point,
      "The vehicle has overrun the front stop point on the trajectory.");
  });
  d.add(ns + "latency", [&](auto & stat) {
    set_status(
      stat, validation_status_.is_valid_latency, "The latency is larger than expected value.");
  });
}

bool ControlValidator::is_data_ready()
{
  const auto waiting = [this](const auto topic_name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", topic_name);
    return false;
  };

  if (!current_kinematics_) {
    return waiting(sub_kinematics_->subscriber()->get_topic_name());
  }
  if (!current_reference_trajectory_) {
    return waiting(sub_reference_traj_->subscriber()->get_topic_name());
  }
  if (!current_predicted_trajectory_) {
    return waiting(sub_reference_traj_->subscriber()->get_topic_name());
  }
  if (!acceleration_msg_) {
    return waiting(sub_measured_acc_->subscriber()->get_topic_name());
  }
  if (!control_cmd_msg_) {
    return waiting(sub_control_cmd_->get_topic_name());
  }
  return true;
}

void ControlValidator::on_control_cmd(const Control::ConstSharedPtr msg)
{
  stop_watch.tic();

  control_cmd_msg_ = msg;
  current_predicted_trajectory_ = sub_predicted_traj_->take_data();
  current_reference_trajectory_ = sub_reference_traj_->take_data();
  current_kinematics_ = sub_kinematics_->take_data();
  acceleration_msg_ = sub_measured_acc_->take_data();

  if (!is_data_ready()) return;

  debug_pose_publisher_->clear_markers();

  validation_status_.latency = (this->now() - msg->stamp).seconds();
  validation_status_.is_valid_latency =
    validation_status_.latency < validation_params_.nominal_latency_threshold;

  validate(
    *current_predicted_trajectory_, *current_reference_trajectory_, *current_kinematics_,
    *control_cmd_msg_, *acceleration_msg_);

  diag_updater_.force_update();

  // for debug
  publish_debug_info();
  display_status();
}

void ControlValidator::publish_debug_info()
{
  pub_status_->publish(validation_status_);

  if (!is_all_valid(validation_status_)) {
    geometry_msgs::msg::Pose front_pose = current_kinematics_->pose.pose;
    shift_pose(front_pose, vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m);
    debug_pose_publisher_->push_virtual_wall(front_pose);
    debug_pose_publisher_->push_warning_msg(front_pose, "INVALID CONTROL");
  }
  debug_pose_publisher_->publish();

  // Publish ProcessingTime
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);
}

void ControlValidator::validate(
  const Trajectory & predicted_trajectory, const Trajectory & reference_trajectory,
  const Odometry & kinematics, const Control & control_cmd,
  const AccelWithCovarianceStamped & measured_acc)
{
  if (predicted_trajectory.points.size() < 2) {
    RCLCPP_DEBUG(get_logger(), "predicted_trajectory size is less than 2. Cannot validate.");
    return;
  }
  if (reference_trajectory.points.size() < 2) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "reference_trajectory size is less than 2. Cannot validate.");
    return;
  }

  validation_status_.stamp = get_clock()->now();
  validation_status_.vehicle_vel = vehicle_vel_.filter(kinematics.twist.twist.linear.x);

  std::tie(
    validation_status_.max_distance_deviation, validation_status_.is_valid_max_distance_deviation) =
    calc_lateral_deviation_status(predicted_trajectory, *current_reference_trajectory_);
  acceleration_validator.validate(validation_status_, kinematics, control_cmd, measured_acc);
  calc_velocity_deviation_status(*current_reference_trajectory_, kinematics);
  calc_stop_point_overrun_status(*current_reference_trajectory_, kinematics);

  validation_status_.invalid_count =
    is_all_valid(validation_status_) ? 0 : validation_status_.invalid_count + 1;
}

std::pair<double, bool> ControlValidator::calc_lateral_deviation_status(
  const Trajectory & predicted_trajectory, const Trajectory & reference_trajectory) const
{
  auto max_distance_deviation =
    calc_max_lateral_distance(reference_trajectory, predicted_trajectory);
  return {
    max_distance_deviation,
    max_distance_deviation <= validation_params_.max_distance_deviation_threshold};
}

void ControlValidator::calc_velocity_deviation_status(
  const Trajectory & reference_trajectory, const Odometry & kinematics)
{
  auto & status = validation_status_;
  const auto & params = validation_params_;
  status.target_vel = target_vel_.filter(
    autoware::motion_utils::calcInterpolatedPoint(reference_trajectory, kinematics.pose.pose)
      .longitudinal_velocity_mps);

  const bool is_rolling_back = std::signbit(status.vehicle_vel * status.target_vel) &&
                               std::abs(status.vehicle_vel) > params.rolling_back_velocity;
  if (
    !hold_velocity_error_until_stop_ || !status.is_rolling_back ||
    std::abs(status.vehicle_vel) < 0.05) {
    status.is_rolling_back = is_rolling_back;
  }

  const bool is_over_velocity =
    std::abs(status.vehicle_vel) >
    std::abs(status.target_vel) * (1.0 + params.over_velocity_ratio) + params.over_velocity_offset;
  if (
    !hold_velocity_error_until_stop_ || !status.is_over_velocity ||
    std::abs(status.vehicle_vel) < 0.05) {
    status.is_over_velocity = is_over_velocity;
  }
}

void ControlValidator::calc_stop_point_overrun_status(
  const Trajectory & reference_trajectory, const Odometry & kinematics)
{
  auto & status = validation_status_;
  const auto & params = validation_params_;

  status.dist_to_stop = [](const Trajectory & traj, const geometry_msgs::msg::Pose & pose) {
    const auto stop_idx_opt = autoware::motion_utils::searchZeroVelocityIndex(traj.points);

    const size_t end_idx = stop_idx_opt ? *stop_idx_opt : traj.points.size() - 1;
    const size_t seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(traj.points, pose);
    const double signed_length_on_traj = autoware::motion_utils::calcSignedArcLength(
      traj.points, pose.position, seg_idx, traj.points.at(end_idx).pose.position,
      std::min(end_idx, traj.points.size() - 2));

    if (std::isnan(signed_length_on_traj)) {
      return 0.0;
    }
    return signed_length_on_traj;
  }(reference_trajectory, kinematics.pose.pose);

  status.nearest_trajectory_vel =
    autoware::motion_utils::calcInterpolatedPoint(reference_trajectory, kinematics.pose.pose)
      .longitudinal_velocity_mps;

  // NOTE: the same velocity threshold as autoware::motion_utils::searchZeroVelocity
  status.has_overrun_stop_point = status.dist_to_stop < -params.overrun_stop_point_dist &&
                                  status.nearest_trajectory_vel < 1e-3 && status.vehicle_vel > 1e-3;
}

bool ControlValidator::is_all_valid(const ControlValidatorStatus & s)
{
  return s.is_valid_max_distance_deviation && s.is_valid_acc && !s.is_rolling_back &&
         !s.is_over_velocity && !s.has_overrun_stop_point;
}

void ControlValidator::display_status()
{
  if (!display_on_terminal_) return;
  rclcpp::Clock clock{RCL_ROS_TIME};

  const auto warn = [this, &clock](const bool status, const std::string & msg) {
    if (!status) {
      RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "%s", msg.c_str());
    }
  };

  const auto & s = validation_status_;

  warn(
    s.is_valid_max_distance_deviation,
    "predicted trajectory is too far from planning trajectory!!");
}

}  // namespace autoware::control_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control_validator::ControlValidator)
