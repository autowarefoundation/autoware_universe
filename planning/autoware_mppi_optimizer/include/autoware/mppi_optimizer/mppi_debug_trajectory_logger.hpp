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

#ifndef AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_TRAJECTORY_LOGGER_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_TRAJECTORY_LOGGER_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <rclcpp/logging.hpp>

#include <tf2/utils.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace autoware::mppi_optimizer
{

/** Prefer $XDG_CACHE_HOME, else $HOME/.cache — never a world-writable path like /tmp. */
inline std::string default_mppi_debug_log_directory()
{
  if (const char * xdg = std::getenv("XDG_CACHE_HOME"); xdg != nullptr && xdg[0] != '\0') {
    return std::string(xdg) + "/autoware/mppi_debug_log";
  }
  if (const char * home = std::getenv("HOME"); home != nullptr && home[0] != '\0') {
    return std::string(home) + "/.cache/autoware/mppi_debug_log";
  }
  return {};
}

/** Ego state used by MPPI at the start of a cycle (for offline replay). */
struct MppiDebugEgoState
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  double v{0.0};
  double accel{0.0};
  double steer{0.0};
};

/**
 * Optional CSV logger for reference / optimized trajectories so
 * mppi_debug_visualizer.py can replay offline via --log-dir.
 *
 * Layout:
 *   <log_dir>/index.csv
 *   <log_dir>/vehicle_params.csv   (once)
 *   <log_dir>/cost_params.csv      (once)
 *   <log_dir>/000000_reference.csv
 *   <log_dir>/000000_optimized.csv
 *   <log_dir>/000000_ego.csv
 *   ...
 *
 * Trajectory CSV columns:
 *   t_from_start_s,x,y,z,yaw,v,a,steer,steer_rate
 * Ego CSV columns:
 *   x,y,z,yaw,v,accel,steer
 */
class MppiDebugTrajectoryLogger
{
public:
  void configure(const bool enable, const std::string & directory)
  {
    enabled_ = enable;
    directory_ = directory;
    frame_id_ = 0;
    index_initialized_ = false;
    params_written_ = false;
    if (!enabled_) {
      return;
    }
    if (directory_.empty()) {
      directory_ = default_mppi_debug_log_directory();
    }
    if (directory_.empty()) {
      RCLCPP_WARN(
        rclcpp::get_logger("mppi_debug_trajectory_logger"),
        "Debug trajectory logging enabled but no writable cache dir "
        "(set debug_trajectory_log_directory or HOME/XDG_CACHE_HOME); disabling.");
      enabled_ = false;
      return;
    }
    std::error_code ec;
    std::filesystem::create_directories(directory_, ec);
    if (ec) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mppi_debug_trajectory_logger"),
        "Failed to create log directory '%s': %s", directory_.c_str(), ec.message().c_str());
      enabled_ = false;
      return;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("mppi_debug_trajectory_logger"),
      "MPPI debug trajectory logging enabled -> %s", directory_.c_str());
  }

  bool enabled() const { return enabled_; }

  void writeParamsOnce(
    const FirstOrderDubinsMppiCostParams & cost, const FirstOrderDubinsMppiVehicleParams & vehicle)
  {
    if (!enabled_ || params_written_) {
      return;
    }
    {
      std::ofstream out(directory_ + "/cost_params.csv");
      if (out) {
        out << "key,value\n";
        out << std::setprecision(9) << std::fixed;
        out << "lambda," << cost.lambda << "\n";
        out << "desired_speed," << cost.desired_speed << "\n";
        out << "speed_coeff," << cost.speed_coeff << "\n";
        out << "track_coeff," << cost.track_coeff << "\n";
        out << "heading_coeff," << cost.heading_coeff << "\n";
        out << "crash_coeff," << cost.crash_coeff << "\n";
        out << "boundary_threshold," << cost.boundary_threshold << "\n";
        out << "boundary_threshold_left," << cost.boundary_threshold_left << "\n";
        out << "boundary_threshold_right," << cost.boundary_threshold_right << "\n";
        out << "accel_cmd_coeff," << cost.accel_cmd_coeff << "\n";
        out << "steer_cmd_coeff," << cost.steer_cmd_coeff << "\n";
        out << "steer_rate_coeff," << cost.steer_rate_coeff << "\n";
        out << "lateral_acceleration_coeff," << cost.lateral_acceleration_coeff << "\n";
        out << "lateral_jerk_coeff," << cost.lateral_jerk_coeff << "\n";
        out << "longitudinal_jerk_coeff," << cost.longitudinal_jerk_coeff << "\n";
        out << "obstacle_collision_margin," << cost.obstacle_collision_margin << "\n";
        out << "goal_pos_coeff," << cost.goal_pos_coeff << "\n";
        out << "goal_speed_coeff," << cost.goal_speed_coeff << "\n";
        out << "goal_yaw_coeff," << cost.goal_yaw_coeff << "\n";
        out << "goal_terminal_scale," << cost.goal_terminal_scale << "\n";
      }
    }
    {
      std::ofstream out(directory_ + "/vehicle_params.csv");
      if (out) {
        out << "key,value\n";
        out << std::setprecision(9) << std::fixed;
        out << "ego_length," << vehicle.ego_length << "\n";
        out << "ego_width," << vehicle.ego_width << "\n";
        out << "ego_axle_to_box_center," << vehicle.ego_axle_to_box_center << "\n";
        out << "wheel_base," << vehicle.wheel_base << "\n";
        out << "max_steer_angle," << vehicle.max_steer_angle << "\n";
        out << "acc_time_constant," << vehicle.acc_time_constant << "\n";
        out << "steer_time_constant," << vehicle.steer_time_constant << "\n";
        out << "steer_rate_lim," << vehicle.steer_rate_lim << "\n";
        out << "vel_rate_lim," << vehicle.vel_rate_lim << "\n";
        out << "acc_time_delay," << vehicle.acc_time_delay << "\n";
        out << "steer_time_delay," << vehicle.steer_time_delay << "\n";
      }
    }
    params_written_ = true;
  }

  void logFrame(
    const autoware_planning_msgs::msg::Trajectory & reference,
    const autoware_planning_msgs::msg::Trajectory & optimized, const MppiDebugEgoState & ego,
    const double baseline_cost = 0.0)
  {
    if (!enabled_) {
      return;
    }

    ensureIndexHeader();
    const std::string frame_tag = formatFrameId(frame_id_);
    const std::string ref_path = directory_ + "/" + frame_tag + "_reference.csv";
    const std::string opt_path = directory_ + "/" + frame_tag + "_optimized.csv";
    const std::string ego_path = directory_ + "/" + frame_tag + "_ego.csv";
    if (!writeTrajectoryCsv(ref_path, reference) || !writeTrajectoryCsv(opt_path, optimized) ||
        !writeEgoCsv(ego_path, ego)) {
      return;
    }

    const auto & stamp = reference.header.stamp.sec != 0 || reference.header.stamp.nanosec != 0
                           ? reference.header.stamp
                           : optimized.header.stamp;
    std::ofstream index(directory_ + "/index.csv", std::ios::app);
    if (!index) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mppi_debug_trajectory_logger"), "Failed to append index.csv in %s",
        directory_.c_str());
      return;
    }
    index << frame_id_ << "," << stamp.sec << "," << stamp.nanosec << "," << baseline_cost << ","
          << reference.points.size() << "," << optimized.points.size() << "\n";
    ++frame_id_;
  }

private:
  static std::string formatFrameId(const uint64_t id)
  {
    std::ostringstream oss;
    oss << std::setw(6) << std::setfill('0') << id;
    return oss.str();
  }

  void ensureIndexHeader()
  {
    if (index_initialized_) {
      return;
    }
    const std::string index_path = directory_ + "/index.csv";
    if (!std::filesystem::exists(index_path)) {
      std::ofstream index(index_path);
      if (index) {
        index << "frame_id,stamp_sec,stamp_nsec,baseline_cost,n_reference,n_optimized\n";
      }
    }
    index_initialized_ = true;
  }

  static bool writeTrajectoryCsv(
    const std::string & path, const autoware_planning_msgs::msg::Trajectory & trajectory)
  {
    std::ofstream out(path);
    if (!out) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mppi_debug_trajectory_logger"), "Failed to write %s", path.c_str());
      return false;
    }
    out << "t_from_start_s,x,y,z,yaw,v,a,steer,steer_rate\n";
    out << std::setprecision(9) << std::fixed;
    for (const auto & point : trajectory.points) {
      const double t = static_cast<double>(point.time_from_start.sec) +
                       static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
      const double yaw = tf2::getYaw(point.pose.orientation);
      out << t << "," << point.pose.position.x << "," << point.pose.position.y << ","
          << point.pose.position.z << "," << yaw << "," << point.longitudinal_velocity_mps << ","
          << point.acceleration_mps2 << "," << point.front_wheel_angle_rad << ","
          << point.heading_rate_rps << "\n";
    }
    return true;
  }

  static bool writeEgoCsv(const std::string & path, const MppiDebugEgoState & ego)
  {
    std::ofstream out(path);
    if (!out) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mppi_debug_trajectory_logger"), "Failed to write %s", path.c_str());
      return false;
    }
    out << "x,y,z,yaw,v,accel,steer\n";
    out << std::setprecision(9) << std::fixed;
    out << ego.x << "," << ego.y << "," << ego.z << "," << ego.yaw << "," << ego.v << ","
        << ego.accel << "," << ego.steer << "\n";
    return true;
  }

  bool enabled_{false};
  bool index_initialized_{false};
  bool params_written_{false};
  std::string directory_;
  uint64_t frame_id_{0};
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_TRAJECTORY_LOGGER_HPP_
