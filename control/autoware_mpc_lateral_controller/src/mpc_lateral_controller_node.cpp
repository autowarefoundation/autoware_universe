// Copyright 2026 TIER IV, Inc.
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

#include "autoware/mpc_lateral_controller/mpc_lateral_controller_node.hpp"

#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

MpcLateralControllerConfig MpcLateralControllerNode::createConfig(rclcpp::Node & node)
{
  const auto dp_int = [&](const std::string & s) { return node.declare_parameter<int>(s); };
  const auto dp_bool = [&](const std::string & s) { return node.declare_parameter<bool>(s); };
  const auto dp_double = [&](const std::string & s) { return node.declare_parameter<double>(s); };

  MpcLateralControllerConfig config;

  config.ctrl_period = node.get_parameter("ctrl_period").as_double();

  auto & p_filt = config.trajectory_filtering;
  p_filt.enable_path_smoothing = dp_bool("enable_path_smoothing");
  p_filt.path_filter_moving_ave_num = dp_int("path_filter_moving_ave_num");
  p_filt.curvature_smoothing_num_traj = dp_int("curvature_smoothing_num_traj");
  p_filt.curvature_smoothing_num_ref_steer = dp_int("curvature_smoothing_num_ref_steer");
  p_filt.traj_resample_dist = dp_double("traj_resample_dist");
  p_filt.extend_trajectory_for_end_yaw_control = dp_bool("extend_trajectory_for_end_yaw_control");

  config.use_steer_prediction = dp_bool("use_steer_prediction");
  const double steer_tau = dp_double("vehicle_model_steer_tau");

  /* stop state parameters */
  config.stop_state_entry_ego_speed = dp_double("stop_state_entry_ego_speed");
  config.stop_state_entry_target_speed = dp_double("stop_state_entry_target_speed");
  config.converged_steer_rad = dp_double("converged_steer_rad");
  config.keep_steer_control_until_converged = dp_bool("keep_steer_control_until_converged");
  config.new_traj_duration_time = dp_double("new_traj_duration_time");            // [s]
  config.new_traj_end_dist = dp_double("new_traj_end_dist");                      // [m]
  config.mpc_converged_threshold_rps = dp_double("mpc_converged_threshold_rps");  // [rad/s]

  /* vehicle */
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  config.wheelbase = vehicle_info.wheel_base_m;
  config.steer_lim = vehicle_info.max_steer_angle_rad;
  constexpr double deg2rad = static_cast<double>(M_PI) / 180.0;

  // steer rate limit depending on curvature
  const auto steer_rate_lim_dps_list_by_curvature =
    node.declare_parameter<std::vector<double>>("steer_rate_lim_dps_list_by_curvature");
  const auto curvature_list_for_steer_rate_lim =
    node.declare_parameter<std::vector<double>>("curvature_list_for_steer_rate_lim");
  for (size_t i = 0; i < steer_rate_lim_dps_list_by_curvature.size(); ++i) {
    config.steer_rate_lim_by_curvature.emplace_back(
      curvature_list_for_steer_rate_lim.at(i),
      steer_rate_lim_dps_list_by_curvature.at(i) * deg2rad);
  }

  // steer rate limit depending on velocity
  const auto steer_rate_lim_dps_list_by_velocity =
    node.declare_parameter<std::vector<double>>("steer_rate_lim_dps_list_by_velocity");
  const auto velocity_list_for_steer_rate_lim =
    node.declare_parameter<std::vector<double>>("velocity_list_for_steer_rate_lim");
  for (size_t i = 0; i < steer_rate_lim_dps_list_by_velocity.size(); ++i) {
    config.steer_rate_lim_by_velocity.emplace_back(
      velocity_list_for_steer_rate_lim.at(i), steer_rate_lim_dps_list_by_velocity.at(i) * deg2rad);
  }

  config.vehicle_model_type = node.declare_parameter<std::string>("vehicle_model_type");
  if (config.vehicle_model_type == "dynamics") {
    config.mass_fl = dp_double("vehicle.mass_fl");
    config.mass_fr = dp_double("vehicle.mass_fr");
    config.mass_rl = dp_double("vehicle.mass_rl");
    config.mass_rr = dp_double("vehicle.mass_rr");
    config.cf = dp_double("vehicle.cf");
    config.cr = dp_double("vehicle.cr");
  }

  config.qp_solver_type = node.declare_parameter<std::string>("qp_solver_type");

  config.input_delay = dp_double("input_delay");

  /* steering offset compensation */
  config.enable_auto_steering_offset_removal =
    dp_bool("steering_offset.enable_auto_steering_offset_removal");
  config.steer_offset_max_update_th = dp_double("steering_offset.max_update_th");
  if (config.enable_auto_steering_offset_removal) {
    config.steer_offset_filter_cutoff_hz =
      dp_double("steering_offset.steer_offset_filter_cutoff_hz");
  }

  config.steering_lpf_cutoff_hz = dp_double("steering_lpf_cutoff_hz");
  config.error_deriv_lpf_cutoff_hz = dp_double("error_deriv_lpf_cutoff_hz");

  // ego nearest index search
  const auto check_and_get_param = [&](const auto & param) {
    return node.has_parameter(param) ? node.get_parameter(param).as_double() : dp_double(param);
  };
  config.ego_nearest_dist_threshold = check_and_get_param("ego_nearest_dist_threshold");
  config.ego_nearest_yaw_threshold = check_and_get_param("ego_nearest_yaw_threshold");

  config.use_delayed_initial_state = dp_bool("use_delayed_initial_state");

  // trajectory_reference_mode is declared at controller_node level
  // If not available, declare it with default value for standalone usage
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

  config.publish_debug_trajectories = dp_bool("publish_debug_trajectories");

  config.mpc_param = declareMPCparameters(node);
  config.mpc_param.steer_tau = steer_tau;

  return config;
}

MpcLateralControllerNode::MpcLateralControllerNode(
  rclcpp::Node & node, std::shared_ptr<diagnostic_updater::Updater> diag_updater)
: clock_(node.get_clock()),
  logger_(node.get_logger().get_child("lateral_controller")),
  writer_(logger_, clock_),
  diag_updater_(diag_updater)
{
  m_controller = std::make_unique<MpcLateralController>(createConfig(node), writer_);

  m_pub_predicted_traj = node.create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
  m_pub_predicted_traj_frenet =
    node.create_publisher<Trajectory>("~/debug/predicted_trajectory_in_frenet_coordinate", 1);
  m_pub_resampled_reference_traj =
    node.create_publisher<Trajectory>("~/debug/resampled_reference_trajectory", 1);
  m_pub_nearest_pose = node.create_publisher<PoseStamped>("~/debug/nearest_pose", 1);
  m_pub_nearest_segment_traj = node.create_publisher<Trajectory>("~/debug/nearest_segment", 1);
  m_pub_nearest_info = node.create_publisher<Float32MultiArrayStamped>("~/debug/nearest_info", 1);
  m_pub_debug_values =
    node.create_publisher<Float32MultiArrayStamped>("~/output/lateral_diagnostic", 1);
  m_pub_steer_offset = node.create_publisher<Float32Stamped>("~/output/estimated_steer_offset", 1);

  /* get parameter updates */
  using std::placeholders::_1;
  m_set_param_res = node.add_on_set_parameters_callback(
    std::bind(&MpcLateralControllerNode::paramCallback, this, _1));

  diag_updater_->add("MPC_solve_checker", [&](auto & stat) { setStatus(stat); });
}

bool MpcLateralControllerNode::isReady(const trajectory_follower::InputData & input_data)
{
  return m_controller->isReady(input_data);
}

trajectory_follower::LateralOutput MpcLateralControllerNode::run(
  trajectory_follower::InputData const & input_data)
{
  // Timestamp of this control cycle, shared by every message this call publishes.
  const auto stamp = clock_->now();

  const auto result = m_controller->run(input_data, stamp);

  publish(result);

  return result.output;
}

void MpcLateralControllerNode::publish(const MpcLateralControllerResult & result)
{
  auto predicted_trajectory = result.mpc.predicted_trajectory;
  m_pub_predicted_traj->publish(predicted_trajectory);

  if (result.mpc.debug_msgs) {
    const auto & debug_msgs = *result.mpc.debug_msgs;
    m_pub_predicted_traj_frenet->publish(debug_msgs.predicted_trajectory_frenet);
    m_pub_resampled_reference_traj->publish(debug_msgs.resampled_reference_trajectory);
    m_pub_nearest_pose->publish(debug_msgs.nearest_pose);
    m_pub_nearest_segment_traj->publish(debug_msgs.nearest_segment_trajectory);
    m_pub_nearest_info->publish(debug_msgs.nearest_info);
  }

  m_pub_debug_values->publish(result.mpc.diagnostic);

  Float32Stamped offset;
  offset.stamp = clock_->now();
  offset.data = static_cast<float>(result.steering_offset);
  m_pub_steer_offset->publish(offset);
}

void MpcLateralControllerNode::setStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto & mpc_result = m_controller->lastMpcResult();
  if (mpc_result.result) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "MPC succeeded.");
  } else {
    const std::string error_msg = "MPC failed due to " + mpc_result.reason;
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_msg);
  }
}

MPCParam MpcLateralControllerNode::declareMPCparameters(rclcpp::Node & node)
{
  MPCParam mpc_param;
  mpc_param.prediction_horizon = node.declare_parameter<int>("mpc_prediction_horizon");
  mpc_param.prediction_dt = node.declare_parameter<double>("mpc_prediction_dt");

  const auto dp = [&](const auto & param) { return node.declare_parameter<double>(param); };

  auto & nw = mpc_param.nominal_weight;
  nw.lat_error = dp("mpc_weight_lat_error");
  nw.heading_error = dp("mpc_weight_heading_error");
  nw.heading_error_squared_vel = dp("mpc_weight_heading_error_squared_vel");
  nw.steering_input = dp("mpc_weight_steering_input");
  nw.steering_input_squared_vel = dp("mpc_weight_steering_input_squared_vel");
  nw.lat_jerk = dp("mpc_weight_lat_jerk");
  nw.steer_rate = dp("mpc_weight_steer_rate");
  nw.steer_acc = dp("mpc_weight_steer_acc");
  nw.terminal_lat_error = dp("mpc_weight_terminal_lat_error");
  nw.terminal_heading_error = dp("mpc_weight_terminal_heading_error");

  auto & lcw = mpc_param.low_curvature_weight;
  lcw.lat_error = dp("mpc_low_curvature_weight_lat_error");
  lcw.heading_error = dp("mpc_low_curvature_weight_heading_error");
  lcw.heading_error_squared_vel = dp("mpc_low_curvature_weight_heading_error_squared_vel");
  lcw.steering_input = dp("mpc_low_curvature_weight_steering_input");
  lcw.steering_input_squared_vel = dp("mpc_low_curvature_weight_steering_input_squared_vel");
  lcw.lat_jerk = dp("mpc_low_curvature_weight_lat_jerk");
  lcw.steer_rate = dp("mpc_low_curvature_weight_steer_rate");
  lcw.steer_acc = dp("mpc_low_curvature_weight_steer_acc");
  mpc_param.low_curvature_thresh_curvature = dp("mpc_low_curvature_thresh_curvature");

  mpc_param.zero_ff_steer_deg = dp("mpc_zero_ff_steer_deg");
  mpc_param.acceleration_limit = dp("mpc_acceleration_limit");
  mpc_param.velocity_time_constant = dp("mpc_velocity_time_constant");
  mpc_param.min_prediction_length = dp("mpc_min_prediction_length");

  return mpc_param;
}

rcl_interfaces::msg::SetParametersResult MpcLateralControllerNode::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  MPCParam param = m_controller->mpcParam();

  using MPCUtils::update_param;
  try {
    auto & nw = param.nominal_weight;
    auto & lcw = param.low_curvature_weight;

    update_param(parameters, "mpc_prediction_horizon", param.prediction_horizon);
    update_param(parameters, "mpc_prediction_dt", param.prediction_dt);

    const std::string ns_nw = "mpc_weight_";
    update_param(parameters, ns_nw + "lat_error", nw.lat_error);
    update_param(parameters, ns_nw + "heading_error", nw.heading_error);
    update_param(parameters, ns_nw + "heading_error_squared_vel", nw.heading_error_squared_vel);
    update_param(parameters, ns_nw + "steering_input", nw.steering_input);
    update_param(parameters, ns_nw + "steering_input_squared_vel", nw.steering_input_squared_vel);
    update_param(parameters, ns_nw + "lat_jerk", nw.lat_jerk);
    update_param(parameters, ns_nw + "steer_rate", nw.steer_rate);
    update_param(parameters, ns_nw + "steer_acc", nw.steer_acc);
    update_param(parameters, ns_nw + "terminal_lat_error", nw.terminal_lat_error);
    update_param(parameters, ns_nw + "terminal_heading_error", nw.terminal_heading_error);

    const std::string ns_lcw = "mpc_low_curvature_weight_";
    update_param(parameters, ns_lcw + "lat_error", lcw.lat_error);
    update_param(parameters, ns_lcw + "heading_error", lcw.heading_error);
    update_param(parameters, ns_lcw + "heading_error_squared_vel", lcw.heading_error_squared_vel);
    update_param(parameters, ns_lcw + "steering_input", lcw.steering_input);
    update_param(parameters, ns_lcw + "steering_input_squared_vel", lcw.steering_input_squared_vel);
    update_param(parameters, ns_lcw + "lat_jerk", lcw.lat_jerk);
    update_param(parameters, ns_lcw + "steer_rate", lcw.steer_rate);
    update_param(parameters, ns_lcw + "steer_acc", lcw.steer_acc);

    update_param(
      parameters, "mpc_low_curvature_thresh_curvature", param.low_curvature_thresh_curvature);

    update_param(parameters, "mpc_zero_ff_steer_deg", param.zero_ff_steer_deg);
    update_param(parameters, "mpc_acceleration_limit", param.acceleration_limit);
    update_param(parameters, "mpc_velocity_time_constant", param.velocity_time_constant);
    update_param(parameters, "mpc_min_prediction_length", param.min_prediction_length);

    update_param(parameters, "input_delay", param.input_delay);

    // transaction succeeds, now assign values
    m_controller->setMpcParam(param);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

}  // namespace autoware::motion::control::mpc_lateral_controller
