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

#include <angles/angles/angles.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;

void LatencyValidator::validate(
  ControlValidatorStatus & res, const Control & control_cmd, rclcpp::Node & node) const
{
  res.latency = (node.now() - control_cmd.stamp).seconds();
  res.is_valid_latency = res.latency < nominal_latency_threshold;
}

void TrajectoryValidator::validate(
  ControlValidatorStatus & res, const Trajectory & predicted_trajectory,
  const Trajectory & reference_trajectory)
{
  // First, check with the current reference_trajectory
  double max_dist_current = calc_max_lateral_distance(reference_trajectory, predicted_trajectory);
  bool is_valid_current = max_dist_current <= max_distance_deviation_threshold;

  // Note: The reason for comparing with the previous reference_trajectory is that
  // the predicted_trajectory of the current cycle may not have been generated based on
  // the reference_trajectory of the current cycle, but rather on the reference_trajectory of the
  // previous cycle. Therefore, also check the deviation with the previous reference_trajectory.
  // Only if the threshold is exceeded, also check with the previous reference_trajectory
  bool is_valid_prev = true;
  double max_dist_prev = 0.0;
  if (!is_valid_current && prev_reference_trajectory_.has_value()) {
    max_dist_prev =
      calc_max_lateral_distance(prev_reference_trajectory_.value(), predicted_trajectory);
    is_valid_prev = max_dist_prev <= max_distance_deviation_threshold;
  }

  // Only if both exceed the threshold, it is judged as abnormal
  res.max_distance_deviation = std::max(max_dist_current, max_dist_prev);
  res.is_valid_max_distance_deviation = is_valid_current || is_valid_prev;

  // Save the previous reference_trajectory only if the timestamp is different from the current one
  if (
    !prev_reference_trajectory_.has_value() ||
    prev_reference_trajectory_->header.stamp != reference_trajectory.header.stamp) {
    prev_reference_trajectory_ = reference_trajectory;
  }
}

void LateralJerkValidator::validate(
  ControlValidatorStatus & res, const Odometry & kinematic_state, const Control & control_cmd,
  const double wheel_base)
{
  const double filtered_velocity = measured_vel_lpf.filter(kinematic_state.twist.twist.linear.x);
  const double steering_cmd = control_cmd.lateral.steering_tire_angle;

  if (!prev_control_cmd_) {
    prev_control_cmd_ = std::make_unique<Control>(control_cmd);
    return;
  }

  // Calculate time difference
  rclcpp::Time current_time(control_cmd.stamp);
  rclcpp::Time prev_time(prev_control_cmd_->stamp);
  const double dt = (current_time - prev_time).seconds();

  // Only perform calculation if the time difference is greater than or equal to 1 msec.
  // This avoids instability due to too small dt and prevents division by zero.
  if (dt < 1e-3) {
    prev_control_cmd_ = std::make_unique<Control>(control_cmd);
    return;
  }

  const double prev_steering_cmd = prev_control_cmd_->lateral.steering_tire_angle;
  const double steering_rate = (steering_cmd - prev_steering_cmd) / dt;

  // Calculate lateral jerk with the formula
  // j_y = (1/L) * [2V * a_x * θ + V^2 * (dθ/dt)]
  //
  // Where:
  // - L: wheel base
  // - V: longitudinal velocity
  // - a_x: longitudinal acceleration (assumed to be zero for constant velocity)
  // - dθ/dt: steering angle rate of change
  //
  // Note: The calculation assumes constant velocity (a_x = 0), so the first term is omitted.
  const double lateral_jerk =
    (1.0 / wheel_base) * (filtered_velocity * filtered_velocity * steering_rate);

  res.steering_rate = steering_rate;
  res.lateral_jerk = lateral_jerk;
  // Note: Assuming left-right symmetry, only considering the magnitude of jerk
  res.is_valid_lateral_jerk = std::abs(lateral_jerk) < lateral_jerk_threshold_;
  if (!res.is_valid_lateral_jerk) {
    RCLCPP_DEBUG(
      logger_, "Lateral jerk is too high. %f > %f", std::abs(lateral_jerk),
      lateral_jerk_threshold_);
    RCLCPP_DEBUG(
      logger_, "steering_cmd: %f, prev_steering_cmd: %f, dt: %f", steering_cmd,
      prev_control_cmd_->lateral.steering_tire_angle, dt);
  }
  prev_control_cmd_ = std::make_unique<Control>(control_cmd);
}

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

void VelocityValidator::validate(
  ControlValidatorStatus & res, const Trajectory & reference_trajectory,
  const Odometry & kinematics)
{
  const double v_vel = vehicle_vel_lpf.filter(kinematics.twist.twist.linear.x);
  const double t_vel = target_vel_lpf.filter(
    autoware::motion_utils::calcInterpolatedPoint(reference_trajectory, kinematics.pose.pose)
      .longitudinal_velocity_mps);

  const bool is_rolling_back =
    std::signbit(v_vel * t_vel) && std::abs(v_vel) > rolling_back_velocity_th;
  if (!hold_velocity_error_until_stop || !res.is_rolling_back || std::abs(v_vel) < 0.05) {
    res.is_rolling_back = is_rolling_back;
  }

  const bool is_over_velocity =
    std::abs(v_vel) > std::abs(t_vel) * (1.0 + over_velocity_ratio_th) + over_velocity_offset_th;
  if (!hold_velocity_error_until_stop || !res.is_over_velocity || std::abs(v_vel) < 0.05) {
    res.is_over_velocity = is_over_velocity;
  }

  res.vehicle_vel = v_vel;
  res.target_vel = t_vel;
}

void OverrunValidator::validate(
  ControlValidatorStatus & res, const Trajectory & reference_trajectory,
  const Odometry & kinematics)
{
  const auto stop_idx_opt =
    autoware::motion_utils::searchZeroVelocityIndex(reference_trajectory.points);
  const size_t end_idx = stop_idx_opt ? *stop_idx_opt : reference_trajectory.points.size() - 1;
  res.dist_to_stop = autoware::motion_utils::calcSignedArcLength(
    reference_trajectory.points, kinematics.pose.pose.position, end_idx);

  res.nearest_trajectory_vel =
    autoware::motion_utils::calcInterpolatedPoint(reference_trajectory, kinematics.pose.pose)
      .longitudinal_velocity_mps;

  /*
  res.dist_to_stop: distance to stop according to the trajectory.
  v_vel * assumed_delay_time : distance ego will travel before starting the limit deceleration.
  v_vel * v_vel / (2.0 * assumed_limit_acc): distance to stop assuming we apply the limit
  deceleration.
  if res.pred_dist_to_stop is negative, it means that we predict we will stop after the stop point
  contained in the trajectory.
  */
  const double v_vel = vehicle_vel_lpf.filter(kinematics.twist.twist.linear.x);
  res.pred_dist_to_stop =
    res.dist_to_stop - v_vel * assumed_delay_time - v_vel * v_vel / (2.0 * assumed_limit_acc);

  // NOTE: the same velocity threshold as autoware::motion_utils::searchZeroVelocity
  if (v_vel < 1e-3) {
    res.has_overrun_stop_point = false;
    res.will_overrun_stop_point = false;
    return;
  }
  res.has_overrun_stop_point =
    res.dist_to_stop < -overrun_stop_point_dist_th && res.nearest_trajectory_vel < 1e-3;
  res.will_overrun_stop_point = res.pred_dist_to_stop < -will_overrun_stop_point_dist_th;
}

void YawValidator::validate(
  ControlValidatorStatus & res, const Trajectory & reference_trajectory,
  const Odometry & kinematics) const
{
  const auto interpolated_trajectory_point =
    motion_utils::calcInterpolatedPoint(reference_trajectory, kinematics.pose.pose);
  res.yaw_deviation = std::abs(angles::shortest_angular_distance(
    tf2::getYaw(interpolated_trajectory_point.pose.orientation),
    tf2::getYaw(kinematics.pose.pose.orientation)));
  res.is_valid_yaw = res.yaw_deviation <= yaw_deviation_error_th_;
  res.is_warn_yaw = res.yaw_deviation > yaw_deviation_warn_th_;
}

ControlValidator::ControlValidator(const rclcpp::NodeOptions & options)
: Node("control_validator", options), vehicle_info_()
{
  using std::placeholders::_1;

  sub_control_cmd_ = create_subscription<Control>(
    "~/input/control_cmd", 1, std::bind(&ControlValidator::on_control_cmd, this, _1));
  sub_operational_state_ =
    autoware_utils::InterProcessPollingSubscriber<OperationModeState>::create_subscription(
      this, "~/input/operational_mode_state", 1);
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
  d.add(ns + "will_overrun_stop_point", [&](auto & stat) {
    set_status(
      stat, !validation_status_.will_overrun_stop_point,
      "In a few seconds ago, the vehicle will overrun the front stop point on the trajectory.");
  });

  d.add(ns + "latency", [&](auto & stat) {
    set_status(
      stat, validation_status_.is_valid_latency, "The latency is larger than expected value.");
  });

  d.add(ns + "steering_rate", [&](auto & stat) {
    set_status(
      stat, validation_status_.is_valid_lateral_jerk,
      "The lateral jerk is larger than expected value.");
  });

  d.add(ns + "yaw_deviation", [&](auto & stat) {
    set_status(
      stat, validation_status_.is_valid_yaw, "The vehicle yaw has deviated from the trajectory.");
    // TODO(someone): implement the dual thresholds for WARN/ERROR for the other metrics
    if (validation_status_.is_valid_yaw && validation_status_.is_warn_yaw) {
      stat.summary(
        DiagnosticStatus::WARN, "The vehicle yaw is deviating but is still under the error value.");
    }
  });
}

bool ControlValidator::infer_autonomous_control_state(const OperationModeState::ConstSharedPtr msg)
{
  return (msg->mode == OperationModeState::AUTONOMOUS) && (msg->is_autoware_control_enabled);
}

void ControlValidator::validation_filtering(ControlValidatorStatus & res)
{
  // Set all boolean status into valid state
  res.is_valid_max_distance_deviation = true;
  res.is_valid_acc = true;
  res.is_rolling_back = false;
  res.is_over_velocity = false;
  res.is_valid_lateral_jerk = true;
  res.has_overrun_stop_point = false;
  res.will_overrun_stop_point = false;
  res.is_valid_latency = true;
  res.is_valid_yaw = true;
  res.is_warn_yaw = false;
}

void ControlValidator::on_control_cmd(const Control::ConstSharedPtr msg)
{
  stop_watch.tic();

  // prepare ros topics
  const auto waiting = [this](const auto topic_name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", topic_name);
    return;
  };

  Control::ConstSharedPtr control_cmd_msg = msg;
  if (!control_cmd_msg) {
    return waiting(sub_control_cmd_->get_topic_name());
  }
  Trajectory::ConstSharedPtr predicted_trajectory_msg = sub_predicted_traj_->take_data();
  if (!predicted_trajectory_msg) {
    return waiting(sub_reference_traj_->subscriber()->get_topic_name());
  }
  Trajectory::ConstSharedPtr reference_trajectory_msg = sub_reference_traj_->take_data();
  if (!reference_trajectory_msg) {
    return waiting(sub_reference_traj_->subscriber()->get_topic_name());
  }
  if (reference_trajectory_msg->points.size() < 2) {
    // TODO(takagi): This check should be moved into each of the individual validate() functions.
    // Passing the rclcpp::Logger as an argument to the validate() function is necessary.
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "reference_trajectory size is less than 2. Cannot validate.");
    return;
  }
  OperationModeState::ConstSharedPtr operation_mode_msg = sub_operational_state_->take_data();
  if (operation_mode_msg) {
    flag_autonomous_control_enabled_ = infer_autonomous_control_state(operation_mode_msg);
  }
  Odometry::ConstSharedPtr kinematics_msg = sub_kinematics_->take_data();
  if (!kinematics_msg) {
    return waiting(sub_kinematics_->subscriber()->get_topic_name());
  }
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_msg = sub_measured_acc_->take_data();
  if (!acceleration_msg) {
    return waiting(sub_measured_acc_->subscriber()->get_topic_name());
  }

  // pre process
  debug_pose_publisher_->clear_markers();
  validation_status_.stamp = get_clock()->now();

  // validation process
  latency_validator.validate(validation_status_, *control_cmd_msg, *this);

  lateral_jerk_validator.validate(
    validation_status_, *kinematics_msg, *control_cmd_msg, vehicle_info_.wheel_base_m);

  if (predicted_trajectory_msg->points.size() < 2) {
    // TODO(takagi): This check should be moved into each of the individual validate() functions.
    // Passing the rclcpp::Logger as an argument to the validate() function is necessary.
    RCLCPP_DEBUG(get_logger(), "predicted_trajectory size is less than 2. Cannot validate.");
  } else {
    trajectory_validator.validate(
      validation_status_, *predicted_trajectory_msg, *reference_trajectory_msg);
  }
  acceleration_validator.validate(
    validation_status_, *kinematics_msg, *control_cmd_msg, *acceleration_msg);
  velocity_validator.validate(validation_status_, *reference_trajectory_msg, *kinematics_msg);
  overrun_validator.validate(validation_status_, *reference_trajectory_msg, *kinematics_msg);
  yaw_validator.validate(validation_status_, *reference_trajectory_msg, *kinematics_msg);

  if (!flag_autonomous_control_enabled_) {
    // if warnings or errors are being suppressed, printing simple logs
    if (!is_all_valid(validation_status_)) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 3000, "Suppressing control validation during manual driving");
    }
    validation_filtering(validation_status_);
  }

  // post process
  validation_status_.invalid_count =
    is_all_valid(validation_status_) ? 0 : validation_status_.invalid_count + 1;

  diag_updater_.force_update();

  publish_debug_info(kinematics_msg->pose.pose);
  display_status();
}

void ControlValidator::publish_debug_info(const geometry_msgs::msg::Pose & ego_pose)
{
  pub_status_->publish(validation_status_);

  if (!is_all_valid(validation_status_)) {
    geometry_msgs::msg::Pose front_pose = ego_pose;
    shift_pose(front_pose, vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m);
    std::string error_message = generate_error_message(validation_status_);
    debug_pose_publisher_->push_virtual_wall(front_pose, error_message);
  }
  debug_pose_publisher_->publish();

  // Publish ProcessingTime
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);
}

bool ControlValidator::is_all_valid(const ControlValidatorStatus & s)
{
  return s.is_valid_lateral_jerk && s.is_valid_max_distance_deviation && s.is_valid_acc &&
         !s.is_rolling_back && !s.is_over_velocity && !s.has_overrun_stop_point &&
         !s.will_overrun_stop_point && s.is_valid_yaw;
}

std::string ControlValidator::generate_error_message(const ControlValidatorStatus & s)
{
  std::vector<std::string> error_messages;

  if (!s.is_valid_lateral_jerk) {
    error_messages.push_back("HIGH LATERAL JERK");
  }

  if (!s.is_valid_max_distance_deviation) {
    error_messages.push_back("TRAJECTORY DEVIATION");
  }

  if (!s.is_valid_acc) {
    error_messages.push_back("ACCELERATION ERROR");
  }

  if (s.is_rolling_back) {
    error_messages.push_back("ROLLING BACK");
  }

  if (s.is_over_velocity) {
    error_messages.push_back("OVER VELOCITY");
  }

  if (s.has_overrun_stop_point) {
    error_messages.push_back("OVERRUN STOP POINT");
  }

  if (s.will_overrun_stop_point) {
    error_messages.push_back("WILL OVERRUN STOP POINT");
  }

  if (error_messages.empty()) {
    return "INVALID CONTROL";
  }

  if (error_messages.size() == 1) {
    return error_messages[0];
  } else {
    std::string result = error_messages[0];
    for (size_t i = 1; i < error_messages.size(); ++i) {
      result += ", " + error_messages[i];
    }
    return result;
  }
}

void ControlValidator::display_status()
{
  if (!display_on_terminal_) return;
  static rclcpp::Clock clock{RCL_ROS_TIME};

  const auto warn = [this](const bool status, const std::string & msg, const double value) {
    if (!status) {
      RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "%s: %.2f", msg.c_str(), value);
    }
  };

  const auto & s = validation_status_;

  warn(
    s.is_valid_max_distance_deviation,
    "predicted trajectory is too far from planning trajectory with max distance deviation: ",
    s.max_distance_deviation);
  warn(
    s.is_valid_lateral_jerk,
    "lateral jerk exceeds safety threshold with steering rate: ", s.steering_rate);
}

}  // namespace autoware::control_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control_validator::ControlValidator)
