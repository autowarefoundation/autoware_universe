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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_temporal_mpt_optimizer.hpp"

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_temporal_mpt_optimizer_utils.hpp"

#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
namespace
{
/** Overlay MPC state (x, y, psi, v) and steering from u[1] on reference points for debug
 * visualization (solver last iterate; reference alone has no optimized steering). */
TrajectoryPoints trajectory_from_solution_overlay(
  const TrajectoryPoints & reference, const temporal_mpt::AcadosSolution & solution, size_t n_out)
{
  TrajectoryPoints out;
  const size_t n = std::min({n_out, reference.size(), static_cast<size_t>(temporal_mpt::N + 1)});
  out.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint p = reference[i];
    p.pose.position.x = solution.xtraj[i][0];
    p.pose.position.y = solution.xtraj[i][1];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, solution.xtraj[i][2]);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    p.longitudinal_velocity_mps = static_cast<float>(std::max(0.0, solution.xtraj[i][3]));
    const size_t uk = std::min(i, static_cast<size_t>(temporal_mpt::N) - 1);
    p.front_wheel_angle_rad = static_cast<float>(solution.utraj[uk][1]);
    if (i < static_cast<size_t>(temporal_mpt::N)) {
      p.acceleration_mps2 = static_cast<float>(solution.utraj[i][0]);
    }
    out.push_back(std::move(p));
  }
  if (out.size() >= 2) {
    const double v0 = std::max(0.0, solution.xtraj[0][3]);
    const double v1 = std::max(0.0, solution.xtraj[1][3]);
    out[0].longitudinal_velocity_mps = static_cast<float>(std::max(v0, v1));
  }
  return out;
}

std::string expand_user_path_string(const std::string & p)
{
  if (p.empty()) {
    return p;
  }
  if (p.front() == '~' && (p.size() == 1 || p[1] == '/')) {
    if (const char * home = std::getenv("HOME")) {
      return std::string(home) + p.substr(1);
    }
  }
  return p;
}
}  // namespace

void TrajectoryTemporalMPTOptimizer::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper)
{
  TrajectoryOptimizerPluginBase::initialize(name, node_ptr, time_keeper);
  // set_up_params() already ran via TrajectoryOptimizerPluginBase::initialize()
  create_or_reset_solver();
}

void TrajectoryTemporalMPTOptimizer::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  // SQP max_iter / tol: from codegen (generators/path_tracking_mpc_temporal.py → acados_ocp.json),
  // applied inside kinematic_bicycle_temporal_acados_create — not overridden in C++.

  mpt_params_.cg_distance_from_rear_axle_ratio = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_temporal_mpt_optimizer.cg_distance_from_rear_axle_ratio");
  update_bicycle_geometry_from_vehicle();

  mpt_params_.min_points_for_optimization = static_cast<size_t>(get_or_declare_parameter<int>(
    *node_ptr, "trajectory_temporal_mpt_optimizer.min_points_for_optimization"));
  mpt_params_.enable_debug_info = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_temporal_mpt_optimizer.enable_debug_info");
  mpt_params_.publish_debug_topics = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_temporal_mpt_optimizer.publish_debug_topics");
  mpt_params_.write_replay_fixture = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_temporal_mpt_optimizer.write_replay_fixture");
  mpt_params_.replay_fixture_directory = get_or_declare_parameter<std::string>(
    *node_ptr, "trajectory_temporal_mpt_optimizer.replay_fixture_directory");
  mpt_params_.log_replay_fixture_to_console = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_temporal_mpt_optimizer.log_replay_fixture_to_console");

  if (mpt_params_.write_replay_fixture && !mpt_params_.replay_fixture_directory.empty()) {
    RCLCPP_INFO(
      node_ptr->get_logger(),
      "Temporal MPT: writing replay fixtures to %s (feed files to "
      "generators/example_trajectory_file_xyv.py)",
      expand_user_path_string(mpt_params_.replay_fixture_directory).c_str());
  }
}

rcl_interfaces::msg::SetParametersResult TrajectoryTemporalMPTOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  int min_points_for_optimization = static_cast<int>(mpt_params_.min_points_for_optimization);
  if (update_param(
        parameters, "trajectory_temporal_mpt_optimizer.min_points_for_optimization",
        min_points_for_optimization)) {
    mpt_params_.min_points_for_optimization =
      static_cast<size_t>(std::max(2, min_points_for_optimization));
  }
  update_param(
    parameters, "trajectory_temporal_mpt_optimizer.enable_debug_info",
    mpt_params_.enable_debug_info);
  update_param(
    parameters, "trajectory_temporal_mpt_optimizer.publish_debug_topics",
    mpt_params_.publish_debug_topics);
  update_param(
    parameters, "trajectory_temporal_mpt_optimizer.write_replay_fixture",
    mpt_params_.write_replay_fixture);
  update_param(
    parameters, "trajectory_temporal_mpt_optimizer.replay_fixture_directory",
    mpt_params_.replay_fixture_directory);
  update_param(
    parameters, "trajectory_temporal_mpt_optimizer.log_replay_fixture_to_console",
    mpt_params_.log_replay_fixture_to_console);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void TrajectoryTemporalMPTOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  TrajectoryOptimizerData & data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *get_time_keeper());
  if (!params.use_temporal_mpt_optimizer) {
    return;
  }

  if (!acados_interface_) {
    create_or_reset_solver();
  }
  if (!acados_interface_) {
    return;
  }

  const size_t min_points = std::max<size_t>(2, mpt_params_.min_points_for_optimization);
  if (traj_points.size() < min_points) {
    return;
  }

  const TrajectoryPoints reference_snapshot = traj_points;

  // Initial state from the first incoming trajectory point (kinematic bicycle: x, y, psi, v).
  const auto & p0 = traj_points.front();
  const std::array<double, temporal_mpt::NX> x0 = {
    p0.pose.position.x, p0.pose.position.y, tf2::getYaw(p0.pose.orientation),
    std::max(0.0, static_cast<double>(p0.longitudinal_velocity_mps))};

  const auto refs =
    trajectory_temporal_mpt_optimizer_utils::build_temporal_mpt_references(traj_points, x0);

  acados_interface_->set_stage_reference(0, refs.yref_stage0);
  for (size_t k = 1; k < temporal_mpt::N; ++k) {
    acados_interface_->set_stage_reference(static_cast<int>(k), refs.stage_yrefs[k - 1]);
  }
  acados_interface_->set_terminal_reference(refs.terminal_yref);

  auto solution = acados_interface_->get_control(refs.x0_local);

  log_debug_info(
    x0, reference_snapshot, solution, refs.start_idx, refs.terminal_idx, data, traj_points);

  if (solution.status != 0) {
    return;
  }

  for (size_t i = 0; i <= temporal_mpt::N; ++i) {
    solution.xtraj[i][0] += refs.x_offset;
    solution.xtraj[i][1] += refs.y_offset;
  }

  const size_t n_apply = std::min(traj_points.size(), static_cast<size_t>(temporal_mpt::N + 1));
  for (size_t i = 0; i < n_apply; ++i) {
    auto & p = traj_points.at(i);
    p.pose.position.x = solution.xtraj[i][0];
    p.pose.position.y = solution.xtraj[i][1];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, solution.xtraj[i][2]);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    p.longitudinal_velocity_mps = static_cast<float>(std::max(0.0, solution.xtraj[i][3]));
    {
      const size_t uk = std::min(i, static_cast<size_t>(temporal_mpt::N) - 1);
      p.front_wheel_angle_rad = static_cast<float>(solution.utraj[uk][1]);
    }
    // Model controls: u[0] = longitudinal acceleration, u[1] = steering angle (see
    // bicycle_model_temporal).
    if (i < static_cast<size_t>(temporal_mpt::N)) {
      p.acceleration_mps2 = static_cast<float>(solution.utraj[i][0]);
    }
  }
}

void TrajectoryTemporalMPTOptimizer::log_debug_info(
  const std::array<double, temporal_mpt::NX> & x0, const TrajectoryPoints & reference_snapshot,
  const temporal_mpt::AcadosSolution & solution, const size_t start_idx, const size_t terminal_idx,
  const TrajectoryOptimizerData & data, const TrajectoryPoints & traj_points)
{
  write_temporal_mpt_replay_fixture(x0, reference_snapshot, solution.status);

  if (mpt_params_.enable_debug_info) {
    log_acados_solve_debug(solution.status, x0, start_idx, terminal_idx, data, traj_points);
  }

  if (mpt_params_.publish_debug_topics) {
    publish_temporal_mpt_debug_io(reference_snapshot, data.current_odometry, solution);
  }
}

void TrajectoryTemporalMPTOptimizer::write_temporal_mpt_replay_fixture(
  const std::array<double, temporal_mpt::NX> & x0, const TrajectoryPoints & reference_trajectory,
  const int acados_status)
{
  if (acados_status == 0) {
    return;
  }

  const std::string dir_raw = mpt_params_.replay_fixture_directory;
  const bool want_file = mpt_params_.write_replay_fixture && !dir_raw.empty();
  const bool want_console = mpt_params_.log_replay_fixture_to_console;
  if (!want_file && !want_console) {
    return;
  }

  std::ostringstream body;
  body
    << "# example_trajectory_file_xyv.py: python3 generators/example_trajectory_file_xyv.py \\\n";
  body << "#   --reference-file <this_file> --N " << temporal_mpt::N << " --dt 0.1\n";
  body << "# x0: x[m] y[m] yaw[rad] v[m/s]; then x, y, yaw[rad], v_ref[m/s] per line (plugin "
          "input).\n";
  body << std::setprecision(std::numeric_limits<double>::max_digits10);
  body << "x0: " << x0[0] << " " << x0[1] << " " << x0[2] << " " << x0[3] << "\n";
  for (const auto & p : reference_trajectory) {
    const double pyaw = tf2::getYaw(p.pose.orientation);
    const double v_ref = std::max(0.0, static_cast<double>(p.longitudinal_velocity_mps));
    body << p.pose.position.x << ", " << p.pose.position.y << ", " << pyaw << ", " << v_ref << "\n";
  }
  const std::string text = body.str();

  rclcpp::Logger logger = get_node_ptr()->get_logger();

  if (want_console) {
    RCLCPP_ERROR(
      logger, "Temporal MPT replay fixture (failed, status=%d):\n%s", acados_status, text.c_str());
  }

  if (!want_file) {
    return;
  }

  const std::string dir_exp = expand_user_path_string(dir_raw);
  std::error_code ec;
  std::filesystem::create_directories(dir_exp, ec);
  if (ec) {
    RCLCPP_WARN(
      logger, "Temporal MPT: could not create replay fixture directory %s: %s", dir_exp.c_str(),
      ec.message().c_str());
    return;
  }

  const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
  const std::filesystem::path out_path =
    std::filesystem::path(dir_exp) / ("temporal_mpt_replay_failed_" + std::to_string(wall_ms) +
                                      "_status" + std::to_string(acados_status) + ".txt");

  std::ofstream out(out_path);
  if (!out) {
    RCLCPP_WARN(logger, "Temporal MPT: could not open replay fixture %s", out_path.c_str());
    return;
  }
  out << text;
  RCLCPP_WARN(
    logger, "Temporal MPT: wrote replay fixture for example_trajectory_file_xyv.py: %s",
    out_path.c_str());
}

void TrajectoryTemporalMPTOptimizer::update_bicycle_geometry_from_vehicle()
{
  auto * const node_ptr = get_node_ptr();
  const auto vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();

  const double ratio = std::clamp(mpt_params_.cg_distance_from_rear_axle_ratio, 0.0, 1.0);
  mpt_params_.cg_distance_from_rear_axle_ratio = ratio;
  mpt_params_.lr = vehicle_info.wheel_base_m * ratio;
  mpt_params_.lf = vehicle_info.wheel_base_m - mpt_params_.lr;

  constexpr double min_segment_m = 1e-3;
  if (mpt_params_.lf < min_segment_m || mpt_params_.lr < min_segment_m) {
    RCLCPP_WARN(
      node_ptr->get_logger(),
      "Temporal MPT: wheel_base=%.3f m with cg_distance_from_rear_axle_ratio=%.3f yields "
      "lf=%.3f lr=%.3f; clamping ratio for a valid bicycle split",
      vehicle_info.wheel_base_m, ratio, mpt_params_.lf, mpt_params_.lr);
    mpt_params_.lr = std::max(min_segment_m, mpt_params_.lr);
    mpt_params_.lf = std::max(min_segment_m, vehicle_info.wheel_base_m - mpt_params_.lr);
  }

  RCLCPP_INFO(
    node_ptr->get_logger(),
    "Temporal MPT: bicycle geometry from vehicle (wheel_base=%.3f m, cg_ratio=%.3f): lf=%.3f m "
    "lr=%.3f m",
    vehicle_info.wheel_base_m, mpt_params_.cg_distance_from_rear_axle_ratio, mpt_params_.lf,
    mpt_params_.lr);
}

void TrajectoryTemporalMPTOptimizer::apply_solver_model_parameters()
{
  if (!acados_interface_) {
    return;
  }
  const std::array<double, temporal_mpt::NP> model_params = {mpt_params_.lf, mpt_params_.lr};
  acados_interface_->set_parameters_all_stages(model_params);
}

void TrajectoryTemporalMPTOptimizer::create_or_reset_solver()
{
  acados_interface_ = std::make_unique<temporal_mpt::AcadosInterface>();
  apply_solver_model_parameters();
}

void TrajectoryTemporalMPTOptimizer::log_acados_solve_debug(
  const int acados_status, const std::array<double, temporal_mpt::NX> & x0, const size_t start_idx,
  const size_t terminal_idx, const TrajectoryOptimizerData & data,
  const TrajectoryPoints & traj_points) const
{
  rclcpp::Node * const node = get_node_ptr();
  const rclcpp::Logger logger = node->get_logger();

  if (acados_status != 0) {
    RCLCPP_WARN_THROTTLE(
      logger, *node->get_clock(), 2000, "Temporal MPT acados solve failed with status %d",
      acados_status);
  } else {
    RCLCPP_DEBUG_THROTTLE(
      logger, *node->get_clock(), 2000, "Temporal MPT acados solve succeeded with status %d",
      acados_status);
    return;
  }

  RCLCPP_DEBUG(logger, "Temporal MPT optimize: plugin=%s", get_name().c_str());

  RCLCPP_DEBUG(
    logger,
    "x0 (first trajectory point): x=%.6f y=%.6f yaw=%.6f v=%.6f | MPC start_idx=%zu "
    "terminal_idx=%zu",
    x0[0], x0[1], x0[2], x0[3], start_idx, terminal_idx);
  RCLCPP_DEBUG(
    logger, "odom twist linear (map/body per msg): x=%.6f y=%.6f z=%.6f",
    data.current_odometry.twist.twist.linear.x, data.current_odometry.twist.twist.linear.y,
    data.current_odometry.twist.twist.linear.z);

  std::ostringstream oss;
  oss << "trajectory points (" << traj_points.size() << "): [";
  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto & p = traj_points[i];
    const double pyaw = tf2::getYaw(p.pose.orientation);
    oss << "(" << p.pose.position.x << ", " << p.pose.position.y << ", " << pyaw << ")";
    if (i + 1 < traj_points.size()) {
      oss << ", ";
    }
  }
  oss << "]";

  RCLCPP_DEBUG(logger, "%s", oss.str().c_str());
}

void TrajectoryTemporalMPTOptimizer::ensure_debug_publishers()
{
  if (!mpt_params_.publish_debug_topics || debug_input_trajectory_pub_) {
    return;
  }
  rclcpp::Node * const n = get_node_ptr();
  // Best effort (typical for high-rate planning). Subscribers must use matching reliability
  // (default rclpy reliable will not receive these messages).
  const auto qos = rclcpp::QoS(10).best_effort();
  debug_input_trajectory_pub_ = n->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/temporal_mpt/input/reference_trajectory", qos);
  debug_input_initial_state_pub_ =
    n->create_publisher<nav_msgs::msg::Odometry>("~/debug/temporal_mpt/input/initial_state", qos);
  debug_output_trajectory_pub_ = n->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/temporal_mpt/output/trajectory", qos);
  debug_solve_status_pub_ =
    n->create_publisher<std_msgs::msg::Int32>("~/debug/temporal_mpt/output/solve_status", qos);
  debug_control_accel_pub_ = n->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/debug/temporal_mpt/output/control_acceleration_mps2", qos);
  debug_control_delta_cmd_pub_ = n->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/debug/temporal_mpt/output/control_delta_cmd_rad", qos);
}

void TrajectoryTemporalMPTOptimizer::publish_temporal_mpt_debug_io(
  const TrajectoryPoints & reference_before, const nav_msgs::msg::Odometry & initial_odom,
  const temporal_mpt::AcadosSolution & solution)
{
  ensure_debug_publishers();
  if (!debug_input_trajectory_pub_) {
    return;
  }

  const size_t n_out = std::min(reference_before.size(), static_cast<size_t>(temporal_mpt::N + 1));
  const TrajectoryPoints trajectory_after =
    trajectory_from_solution_overlay(reference_before, solution, n_out);

  rclcpp::Node * const node = get_node_ptr();
  std_msgs::msg::Header header;
  header.stamp = node->now();
  header.frame_id = initial_odom.header.frame_id.empty() ? "map" : initial_odom.header.frame_id;

  autoware_planning_msgs::msg::Trajectory input_traj;
  input_traj.header = header;
  input_traj.points = reference_before;

  autoware_planning_msgs::msg::Trajectory output_traj;
  output_traj.header = header;
  output_traj.points = trajectory_after;

  std_msgs::msg::Int32 status_msg;
  status_msg.data = solution.status;

  // Publish u = [a_long, delta] for every stage whenever the solver returned a trajectory,
  // including failed solves (status != 0): values are the last SQP iterate and are useful for
  // debug.
  std_msgs::msg::Float64MultiArray accel_msg;
  std_msgs::msg::Float64MultiArray delta_cmd_msg;
  accel_msg.data.reserve(temporal_mpt::N);
  delta_cmd_msg.data.reserve(temporal_mpt::N);
  for (const auto & u : solution.utraj) {
    accel_msg.data.push_back(u[0]);
    delta_cmd_msg.data.push_back(u[1]);
  }

  debug_input_trajectory_pub_->publish(std::move(input_traj));
  debug_input_initial_state_pub_->publish(initial_odom);
  debug_output_trajectory_pub_->publish(std::move(output_traj));
  debug_solve_status_pub_->publish(std::move(status_msg));
  if (debug_control_accel_pub_) {
    debug_control_accel_pub_->publish(std::move(accel_msg));
  }
  if (debug_control_delta_cmd_pub_) {
    debug_control_delta_cmd_pub_->publish(std::move(delta_cmd_msg));
  }
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectoryTemporalMPTOptimizer,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
