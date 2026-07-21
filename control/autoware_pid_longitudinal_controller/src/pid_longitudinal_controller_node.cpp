// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller_node.hpp"

#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
namespace
{
PidLongitudinalControllerConfig create_config(rclcpp::Node & node)
{
  PidLongitudinalControllerConfig config{};

  // parameters timer
  config.longitudinal_ctrl_period = node.get_parameter("ctrl_period").as_double();

  const auto trajectory_reference_mode =
    node.has_parameter("trajectory_reference_mode")
      ? node.get_parameter("trajectory_reference_mode").as_string()
      : node.declare_parameter<std::string>("trajectory_reference_mode", "spatial");
  if (trajectory_reference_mode == "temporal") {
    config.use_temporal_trajectory = true;
  } else if (trajectory_reference_mode == "spatial") {
    config.use_temporal_trajectory = false;
  } else {
    throw std::invalid_argument(
      "Invalid trajectory_reference_mode. Expected \"spatial\" or \"temporal\".");
  }

  config.wheel_base =
    autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo().wheel_base_m;
  config.front_overhang =
    autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo().front_overhang_m;

  // parameters for delay compensation
  config.delay_compensation_time =
    node.declare_parameter<double>("delay_compensation_time");  // [s]

  // parameters to enable functions
  config.enable_smooth_stop = node.declare_parameter<bool>("enable_smooth_stop");
  config.enable_overshoot_emergency = node.declare_parameter<bool>("enable_overshoot_emergency");
  config.enable_large_tracking_error_emergency =
    node.declare_parameter<bool>("enable_large_tracking_error_emergency");
  config.enable_slope_compensation = node.declare_parameter<bool>("enable_slope_compensation");
  config.enable_keep_stopped_until_steer_convergence =
    node.declare_parameter<bool>("enable_keep_stopped_until_steer_convergence");

  // parameters for state transition
  {
    auto & p = config.state_transition_params;
    // drive
    p.drive_state_stop_dist = node.declare_parameter<double>("drive_state_stop_dist");  // [m]
    p.drive_state_offset_stop_dist =
      node.declare_parameter<double>("drive_state_offset_stop_dist");  // [m]
    // stopping
    p.stopping_state_stop_dist = node.declare_parameter<double>("stopping_state_stop_dist");  // [m]
    p.stopped_state_entry_duration_time =
      node.declare_parameter<double>("stopped_state_entry_duration_time");  // [s]
    // stop
    p.stopped_state_entry_vel = node.declare_parameter<double>("stopped_state_entry_vel");  // [m/s]
    p.stopped_state_entry_acc =
      node.declare_parameter<double>("stopped_state_entry_acc");  // [m/s²]

    // emergency
    p.emergency_state_overshoot_stop_dist =
      node.declare_parameter<double>("emergency_state_overshoot_stop_dist");  // [m]
  }

  // parameters for drive state
  {
    // initialize PID gain
    auto & gains = config.pid_gains;
    gains.kp = node.declare_parameter<double>("kp");
    gains.ki = node.declare_parameter<double>("ki");
    gains.kd = node.declare_parameter<double>("kd");

    // initialize PID limits
    auto & limits = config.pid_limits;
    limits.max_out = node.declare_parameter<double>("max_out");            // [m/s^2]
    limits.min_out = node.declare_parameter<double>("min_out");            // [m/s^2]
    limits.max_p_effort = node.declare_parameter<double>("max_p_effort");  // [m/s^2]
    limits.min_p_effort = node.declare_parameter<double>("min_p_effort");  // [m/s^2]
    limits.max_i_effort = node.declare_parameter<double>("max_i_effort");  // [m/s^2]
    limits.min_i_effort = node.declare_parameter<double>("min_i_effort");  // [m/s^2]
    limits.max_d_effort = node.declare_parameter<double>("max_d_effort");  // [m/s^2]
    limits.min_d_effort = node.declare_parameter<double>("min_d_effort");  // [m/s^2]

    // set lowpass filter for vel error and pitch
    config.lpf_vel_error_gain = node.declare_parameter<double>("lpf_vel_error_gain");

    config.enable_integration_at_low_speed =
      node.declare_parameter<bool>("enable_integration_at_low_speed");
    config.current_vel_threshold_pid_integrate =
      node.declare_parameter<double>("current_vel_threshold_pid_integration");  // [m/s]

    config.time_threshold_before_pid_integrate =
      node.declare_parameter<double>("time_threshold_before_pid_integration");  // [s]
    config.ff_scale_min = node.declare_parameter<double>("ff_scale_min");
    config.ff_scale_max = node.declare_parameter<double>("ff_scale_max");

    config.enable_brake_keeping_before_stop =
      node.declare_parameter<bool>("enable_brake_keeping_before_stop");              // [-]
    config.brake_keeping_acc = node.declare_parameter<double>("brake_keeping_acc");  // [m/s^2]
  }

  // parameters for smooth stop state
  {
    auto & p = config.smooth_stop_params;
    p.max_strong_acc = node.declare_parameter<double>("smooth_stop_max_strong_acc");    // [m/s^2]
    p.min_strong_acc = node.declare_parameter<double>("smooth_stop_min_strong_acc");    // [m/s^2]
    p.weak_acc = node.declare_parameter<double>("smooth_stop_weak_acc");                // [m/s^2]
    p.weak_stop_acc = node.declare_parameter<double>("smooth_stop_weak_stop_acc");      // [m/s^2]
    p.strong_stop_acc = node.declare_parameter<double>("smooth_stop_strong_stop_acc");  // [m/s^2]

    p.min_fast_vel = node.declare_parameter<double>("smooth_stop_max_fast_vel");        // [m/s]
    p.min_running_vel = node.declare_parameter<double>("smooth_stop_min_running_vel");  // [m/s]
    p.min_running_acc = node.declare_parameter<double>("smooth_stop_min_running_acc");  // [m/s^2]
    p.weak_stop_time = node.declare_parameter<double>("smooth_stop_weak_stop_time");    // [s]

    p.weak_stop_dist = node.declare_parameter<double>("smooth_stop_weak_stop_dist");      // [m]
    p.strong_stop_dist = node.declare_parameter<double>("smooth_stop_strong_stop_dist");  // [m]
  }

  // parameters for stop state
  {
    auto & p = config.stopped_state_params;
    p.vel = node.declare_parameter<double>("stopped_vel");  // [m/s]
    p.acc = node.declare_parameter<double>("stopped_acc");  // [m/s^2]
  }

  // parameters for emergency state
  {
    auto & p = config.emergency_state_params;
    p.vel = node.declare_parameter<double>("emergency_vel");    // [m/s]
    p.acc = node.declare_parameter<double>("emergency_acc");    // [m/s^2]
    p.jerk = node.declare_parameter<double>("emergency_jerk");  // [m/s^3]
  }

  // parameters for acc feedback
  {
    config.lpf_acc_error_gain = node.declare_parameter<double>("lpf_acc_error_gain");
    config.acc_feedback_gain = node.declare_parameter<double>("acc_feedback_gain");
  }

  // parameters for acceleration limit
  config.max_acc = node.declare_parameter<double>("max_acc");  // [m/s^2]
  config.min_acc = node.declare_parameter<double>("min_acc");  // [m/s^2]

  // parameters for jerk limit
  config.max_jerk = node.declare_parameter<double>("max_jerk");                  // [m/s^3]
  config.min_jerk = node.declare_parameter<double>("min_jerk");                  // [m/s^3]
  config.max_acc_cmd_diff = node.declare_parameter<double>("max_acc_cmd_diff");  // [m/s^3]

  // parameters for slope compensation
  config.adaptive_trajectory_velocity_th =
    node.declare_parameter<double>("adaptive_trajectory_velocity_th");  // [m/s^2]
  config.lpf_pitch_gain = node.declare_parameter<double>("lpf_pitch_gain");
  config.max_pitch_rad = node.declare_parameter<double>("max_pitch_rad");  // [rad]
  config.min_pitch_rad = node.declare_parameter<double>("min_pitch_rad");  // [rad]

  // check slope source is proper
  const std::string slope_source = node.declare_parameter<std::string>(
    "slope_source");  // raw_pitch, trajectory_pitch or trajectory_adaptive
  if (slope_source == "raw_pitch") {
    config.slope_source = PidLongitudinalControllerConfig::SlopeSource::RAW_PITCH;
  } else if (slope_source == "trajectory_pitch") {
    config.slope_source = PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_PITCH;
  } else if (slope_source == "trajectory_adaptive") {
    config.slope_source = PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_ADAPTIVE;
  } else if (slope_source == "trajectory_goal_adaptive") {
    config.slope_source = PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_GOAL_ADAPTIVE;
  } else {
    RCLCPP_ERROR(node.get_logger(), "Slope source is not valid. Using raw_pitch option as default");
    config.slope_source = PidLongitudinalControllerConfig::SlopeSource::RAW_PITCH;
  }

  // ego nearest index search
  config.ego_nearest_dist_threshold =
    node.has_parameter("ego_nearest_dist_threshold")
      ? node.get_parameter("ego_nearest_dist_threshold").as_double()
      : node.declare_parameter<double>("ego_nearest_dist_threshold");  // [m]
  config.ego_nearest_yaw_threshold =
    node.has_parameter("ego_nearest_yaw_threshold")
      ? node.get_parameter("ego_nearest_yaw_threshold").as_double()
      : node.declare_parameter<double>("ego_nearest_yaw_threshold");  // [rad]

  return config;
}
}  // namespace

PidLongitudinalControllerNode::PidLongitudinalControllerNode(
  rclcpp::Node & node, std::shared_ptr<diagnostic_updater::Updater> diag_updater)
: node_parameters_(node.get_node_parameters_interface()),
  clock_(node.get_clock()),
  logger_(node.get_logger().get_child("longitudinal_controller")),
  config(create_config(node)),
  controller_(config)
{
  using std::placeholders::_1;

  diag_updater_ = diag_updater;

  // subscriber, publisher
  m_pub_slope = node.create_publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(
    "~/output/slope_angle", rclcpp::QoS{1});
  m_pub_debug = node.create_publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(
    "~/output/longitudinal_diagnostic", rclcpp::QoS{1});
  m_pub_virtual_wall_marker = node.create_publisher<MarkerArray>("~/virtual_wall", 1);

  // set parameter callback
  m_set_param_res = node.add_on_set_parameters_callback(
    std::bind(&PidLongitudinalControllerNode::paramCallback, this, _1));

  // diagnostic
  setupDiagnosticUpdater();
}

rcl_interfaces::msg::SetParametersResult PidLongitudinalControllerNode::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto update_param = [&](const std::string & name, double & v) {
    auto it = std::find_if(
      parameters.cbegin(), parameters.cend(),
      [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
    if (it != parameters.cend()) {
      v = it->as_double();
      return true;
    }
    return false;
  };

  // delay compensation
  update_param("delay_compensation_time", config.delay_compensation_time);

  // state transition
  {
    auto & p = config.state_transition_params;
    update_param("drive_state_stop_dist", p.drive_state_stop_dist);
    update_param("stopping_state_stop_dist", p.stopping_state_stop_dist);
    update_param("stopped_state_entry_duration_time", p.stopped_state_entry_duration_time);
    update_param("stopped_state_entry_vel", p.stopped_state_entry_vel);
    update_param("stopped_state_entry_acc", p.stopped_state_entry_acc);
    update_param("emergency_state_overshoot_stop_dist", p.emergency_state_overshoot_stop_dist);
  }

  // drive state
  {
    auto & gains = config.pid_gains;
    update_param("kp", gains.kp);
    update_param("ki", gains.ki);
    update_param("kd", gains.kd);

    update_param("lpf_vel_error_gain", config.lpf_vel_error_gain);

    auto & limits = config.pid_limits;
    update_param("max_out", limits.max_out);
    update_param("min_out", limits.min_out);
    update_param("max_p_effort", limits.max_p_effort);
    update_param("min_p_effort", limits.min_p_effort);
    update_param("max_i_effort", limits.max_i_effort);
    update_param("min_i_effort", limits.min_i_effort);
    update_param("max_d_effort", limits.max_d_effort);
    update_param("min_d_effort", limits.min_d_effort);

    update_param(
      "current_vel_threshold_pid_integration", config.current_vel_threshold_pid_integrate);
    update_param(
      "time_threshold_before_pid_integration", config.time_threshold_before_pid_integrate);
    update_param("ff_scale_min", config.ff_scale_min);
    update_param("ff_scale_max", config.ff_scale_max);
  }

  // stopping state
  {
    auto & p = config.smooth_stop_params;
    update_param("smooth_stop_max_strong_acc", p.max_strong_acc);
    update_param("smooth_stop_min_strong_acc", p.min_strong_acc);
    update_param("smooth_stop_weak_acc", p.weak_acc);
    update_param("smooth_stop_weak_stop_acc", p.weak_stop_acc);
    update_param("smooth_stop_strong_stop_acc", p.strong_stop_acc);
    update_param("smooth_stop_max_fast_vel", p.min_fast_vel);
    update_param("smooth_stop_min_running_vel", p.min_running_vel);
    update_param("smooth_stop_min_running_acc", p.min_running_acc);
    update_param("smooth_stop_weak_stop_time", p.weak_stop_time);
    update_param("smooth_stop_weak_stop_dist", p.weak_stop_dist);
    update_param("smooth_stop_strong_stop_dist", p.strong_stop_dist);
  }

  // stop state
  {
    auto & p = config.stopped_state_params;
    update_param("stopped_vel", p.vel);
    update_param("stopped_acc", p.acc);
  }

  // emergency state
  {
    auto & p = config.emergency_state_params;
    update_param("emergency_vel", p.vel);
    update_param("emergency_acc", p.acc);
    update_param("emergency_jerk", p.jerk);
  }

  // acceleration feedback
  update_param("acc_feedback_gain", config.acc_feedback_gain);
  update_param("lpf_acc_error_gain", config.lpf_acc_error_gain);

  // acceleration limit
  update_param("min_acc", config.min_acc);

  // jerk limit
  update_param("max_jerk", config.max_jerk);
  update_param("min_jerk", config.min_jerk);
  update_param("max_acc_cmd_diff", config.max_acc_cmd_diff);

  // slope compensation
  update_param("max_pitch_rad", config.max_pitch_rad);
  update_param("min_pitch_rad", config.min_pitch_rad);
  update_param("adaptive_trajectory_velocity_th", config.adaptive_trajectory_velocity_th);

  controller_.setConfig(config);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

bool PidLongitudinalControllerNode::isReady(
  [[maybe_unused]] const trajectory_follower::InputData & input_data)
{
  return true;
}

trajectory_follower::LongitudinalOutput PidLongitudinalControllerNode::run(
  trajectory_follower::InputData const & input_data)
{
  // capture the time once for this control cycle
  const rclcpp::Time current_time = clock_->now();

  const auto result =
    controller_.run(input_data, current_time, lateral_sync_data_.is_steer_converged);

  control_state_ = result.control_state;

  emitLogs(result);
  publishDebugData(result, current_time);
  publishVirtualWallMarker(result);

  return result.output;
}

void PidLongitudinalControllerNode::emitLogs(const PidLongitudinalControllerResult & result)
{
  if (result.received_invalid_trajectory) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 3000, "received invalid trajectory. ignore.");
  }
  if (result.emergency_stop_reason) {
    RCLCPP_ERROR(logger_, "Emergency Stop since %s", result.emergency_stop_reason->c_str());
  }
}

void PidLongitudinalControllerNode::publishDebugData(
  const PidLongitudinalControllerResult & result, const rclcpp::Time & current_time)
{
  autoware_internal_debug_msgs::msg::Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = current_time;
  for (const auto & v : controller_.getDebugValues().getValues()) {
    debug_msg.data.push_back(static_cast<decltype(debug_msg.data)::value_type>(v));
  }
  m_pub_debug->publish(debug_msg);

  // slope angle
  autoware_internal_debug_msgs::msg::Float32MultiArrayStamped slope_msg{};
  slope_msg.stamp = current_time;
  slope_msg.data.push_back(static_cast<decltype(slope_msg.data)::value_type>(result.slope_angle));
  m_pub_slope->publish(slope_msg);
}

void PidLongitudinalControllerNode::publishVirtualWallMarker(
  const PidLongitudinalControllerResult & result)
{
  if (!result.virtual_wall_marker) {
    return;
  }
  m_pub_virtual_wall_marker->publish(*result.virtual_wall_marker);
}

void PidLongitudinalControllerNode::setupDiagnosticUpdater()
{
  diag_updater_->add("control_state", this, &PidLongitudinalControllerNode::checkControlState);
}

void PidLongitudinalControllerNode::checkControlState(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (control_state_ == ControlState::EMERGENCY) {
    level = DiagnosticStatus::ERROR;
    msg = "emergency occurred due to ";
  }

  stat.add<int32_t>("control_state", static_cast<int32_t>(control_state_));
  stat.summary(level, msg);
}

}  // namespace autoware::motion::control::pid_longitudinal_controller
