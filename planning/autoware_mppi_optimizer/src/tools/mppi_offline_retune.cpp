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

/**
 * Offline MPPI retune tool.
 *
 * Replays logged diffusion references from enable_debug_trajectory_log CSVs through
 * FirstOrderDubinsMppiInterface with optionally overridden cost params, and writes
 * retuned optimized trajectories for mppi_offline_tuner.py / mppi_debug_visualizer.py.
 *
 * Example:
 *   ros2 run autoware_mppi_optimizer mppi_offline_retune -- \
 *     --log-dir "$HOME/.cache/autoware/mppi_debug_log" \
 *     --out-dir "$HOME/.cache/autoware/mppi_retune" \
 *     --frame 12 --set track_coeff=2000 --set steer_rate_coeff=5000
 */

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"
#include "autoware/mppi_optimizer/mppi_debug_trajectory_io.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/utils.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

using autoware::mppi_optimizer::FirstOrderDubinsMppiCostParams;
using autoware::mppi_optimizer::FirstOrderDubinsMppiInterface;
using autoware::mppi_optimizer::FirstOrderDubinsMppiVehicleParams;
using autoware::mppi_optimizer::formatMppiDebugFrameId;
using autoware::mppi_optimizer::listMppiDebugFrameIds;
using autoware::mppi_optimizer::loadMppiDebugEgoCsv;
using autoware::mppi_optimizer::loadMppiDebugKeyValueCsv;
using autoware::mppi_optimizer::loadMppiDebugTrajectoryCsv;
using autoware::mppi_optimizer::MppiDebugEgoState;
using autoware::mppi_optimizer::writeMppiDebugCostsCsv;
using autoware::mppi_optimizer::writeMppiDebugTrajectoryCsv;
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

void printUsage(const char * argv0)
{
  std::cerr
    << "Usage: " << argv0
    << " --log-dir DIR --out-dir DIR [options]\n"
       "  --log-dir DIR          Input log directory (index.csv + *_reference.csv)\n"
       "  --out-dir DIR          Output directory for retuned CSVs\n"
       "  --frame N              Only retune frame N (default: all)\n"
       "  --params-yaml FILE     Optional ROS-style yaml with cost params\n"
       "  --set key=value        Override a cost param (repeatable)\n"
       "  --wheel-base M         Vehicle wheel base [m] (default 4.76)\n"
       "  --ego-length M         Ego length [m] (default 5.0)\n"
       "  --ego-width M          Ego width [m] (default 1.9)\n"
       "  --max-steer-angle RAD  Max steer [rad] (default 0.7)\n"
       "  --steer-tau S          Steer time constant [s] (default 0.27)\n"
       "  --copy-reference       Also copy reference CSVs into out-dir\n"
       "\n"
       "Loads cost_params.csv / vehicle_params.csv / *_ego.csv from --log-dir when present.\n"
       "CLI --set and vehicle flags override those. Without *_ego.csv, ego IC falls back to\n"
       "reference[0] (will not match online MPPI).\n"
       "  --help\n";
}

std::string trim(const std::string & s)
{
  const auto start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

bool parseKeyValue(const std::string & token, std::string & key, std::string & value)
{
  const auto eq = token.find('=');
  if (eq == std::string::npos) {
    return false;
  }
  key = trim(token.substr(0, eq));
  value = trim(token.substr(eq + 1));
  return !key.empty() && !value.empty();
}

void applyCostParam(
  FirstOrderDubinsMppiCostParams & params, const std::string & key, const float value)
{
  if (key == "lambda") {
    params.lambda = value;
  } else if (key == "desired_speed") {
    params.desired_speed = value;
  } else if (key == "speed_coeff") {
    params.speed_coeff = value;
  } else if (key == "track_coeff") {
    params.track_coeff = value;
  } else if (key == "heading_coeff") {
    params.heading_coeff = value;
  } else if (key == "crash_coeff") {
    params.crash_coeff = value;
  } else if (key == "boundary_threshold") {
    params.boundary_threshold = value;
  } else if (key == "boundary_threshold_left") {
    params.boundary_threshold_left = value;
  } else if (key == "boundary_threshold_right") {
    params.boundary_threshold_right = value;
  } else if (key == "accel_cmd_coeff") {
    params.accel_cmd_coeff = value;
  } else if (key == "steer_cmd_coeff") {
    params.steer_cmd_coeff = value;
  } else if (key == "steer_rate_coeff") {
    params.steer_rate_coeff = value;
  } else if (key == "lateral_acceleration_coeff") {
    params.lateral_acceleration_coeff = value;
  } else if (key == "lateral_jerk_coeff") {
    params.lateral_jerk_coeff = value;
  } else if (key == "longitudinal_jerk_coeff") {
    params.longitudinal_jerk_coeff = value;
  } else if (key == "obstacle_collision_margin") {
    params.obstacle_collision_margin = value;
  } else if (key == "goal_pos_coeff") {
    params.goal_pos_coeff = value;
  } else if (key == "goal_speed_coeff") {
    params.goal_speed_coeff = value;
  } else if (key == "goal_yaw_coeff") {
    params.goal_yaw_coeff = value;
  } else if (key == "goal_terminal_scale") {
    params.goal_terminal_scale = value;
  } else {
    throw std::runtime_error("Unknown cost param: " + key);
  }
}

void loadParamsYaml(const std::string & path, FirstOrderDubinsMppiCostParams & params)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("Failed to open params yaml: " + path);
  }
  std::string line;
  while (std::getline(in, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#' || line.find(':') == std::string::npos) {
      continue;
    }
    const auto colon = line.find(':');
    std::string key = trim(line.substr(0, colon));
    std::string value = trim(line.substr(colon + 1));
    if (!value.empty() && value.front() == '"' && value.back() == '"') {
      value = value.substr(1, value.size() - 2);
    }
    // Skip non-numeric / bool flags.
    if (
      key == "enable_debug_trajectory_log" || key == "debug_trajectory_log_directory" ||
      key == "ros__parameters" || key == "/**") {
      continue;
    }
    try {
      applyCostParam(params, key, std::stof(value));
    } catch (const std::exception &) {
      // Ignore unknown / non-float keys in the yaml.
    }
  }
}

void applyVehicleParam(
  FirstOrderDubinsMppiVehicleParams & params, const std::string & key, const float value)
{
  if (key == "ego_length") {
    params.ego_length = value;
  } else if (key == "ego_width") {
    params.ego_width = value;
  } else if (key == "ego_axle_to_box_center") {
    params.ego_axle_to_box_center = value;
  } else if (key == "wheel_base") {
    params.wheel_base = value;
  } else if (key == "max_steer_angle") {
    params.max_steer_angle = value;
  } else if (key == "acc_time_constant") {
    params.acc_time_constant = value;
  } else if (key == "steer_time_constant") {
    params.steer_time_constant = value;
  } else if (key == "steer_rate_lim") {
    params.steer_rate_lim = value;
  } else if (key == "vel_rate_lim") {
    params.vel_rate_lim = value;
  } else if (key == "acc_time_delay") {
    params.acc_time_delay = value;
  } else if (key == "steer_time_delay") {
    params.steer_time_delay = value;
  } else {
    throw std::runtime_error("Unknown vehicle param: " + key);
  }
}

void loadCostParamsFromLog(const std::string & log_dir, FirstOrderDubinsMppiCostParams & params)
{
  std::unordered_map<std::string, float> kv;
  if (!loadMppiDebugKeyValueCsv(log_dir + "/cost_params.csv", kv)) {
    return;
  }
  for (const auto & [key, value] : kv) {
    try {
      applyCostParam(params, key, value);
    } catch (const std::exception &) {
      // Ignore unknown keys.
    }
  }
  std::cerr << "Loaded cost params from " << log_dir << "/cost_params.csv\n";
}

void loadVehicleParamsFromLog(
  const std::string & log_dir, FirstOrderDubinsMppiVehicleParams & params)
{
  std::unordered_map<std::string, float> kv;
  if (!loadMppiDebugKeyValueCsv(log_dir + "/vehicle_params.csv", kv)) {
    return;
  }
  for (const auto & [key, value] : kv) {
    try {
      applyVehicleParam(params, key, value);
    } catch (const std::exception &) {
      // Ignore unknown keys.
    }
  }
  std::cerr << "Loaded vehicle params from " << log_dir << "/vehicle_params.csv\n";
}

Odometry odometryFromEgo(const MppiDebugEgoState & ego, const Trajectory & reference)
{
  Odometry odom;
  odom.header = reference.header;
  if (odom.header.frame_id.empty()) {
    odom.header.frame_id = "map";
  }
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = ego.x;
  odom.pose.pose.position.y = ego.y;
  odom.pose.pose.position.z = ego.z;
  odom.pose.pose.orientation = autoware::mppi_optimizer::quaternionFromYaw(ego.yaw);
  odom.twist.twist.linear.x = ego.v;
  return odom;
}

geometry_msgs::msg::AccelWithCovarianceStamped accelFromEgo(
  const MppiDebugEgoState & ego, const Trajectory & reference)
{
  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.header = reference.header;
  accel.accel.accel.linear.x = ego.accel;
  return accel;
}

autoware_vehicle_msgs::msg::SteeringReport steeringFromEgo(
  const MppiDebugEgoState & ego, const Trajectory & reference)
{
  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp = reference.header.stamp;
  steering.steering_tire_angle = static_cast<float>(ego.steer);
  return steering;
}

Odometry odometryFromReference(const Trajectory & reference)
{
  Odometry odom;
  odom.header = reference.header;
  if (reference.header.frame_id.empty()) {
    odom.header.frame_id = "map";
  }
  odom.child_frame_id = "base_link";
  const auto & p0 = reference.points.front();
  odom.pose.pose = p0.pose;
  odom.twist.twist.linear.x = p0.longitudinal_velocity_mps;
  return odom;
}

geometry_msgs::msg::AccelWithCovarianceStamped accelFromReference(const Trajectory & reference)
{
  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.header = reference.header;
  accel.accel.accel.linear.x = reference.points.front().acceleration_mps2;
  return accel;
}

autoware_vehicle_msgs::msg::SteeringReport steeringFromReference(const Trajectory & reference)
{
  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp = reference.header.stamp;
  steering.steering_tire_angle = reference.points.front().front_wheel_angle_rad;
  return steering;
}

}  // namespace

int run(int argc, char ** argv);

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << "mppi_offline_retune failed: " << e.what() << "\n";
    return 1;
  }
}

int run(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    }
  }

  struct RclcppGuard
  {
    RclcppGuard(int argc_in, char ** argv_in) { rclcpp::init(argc_in, argv_in); }
    ~RclcppGuard()
    {
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }
  } rclcpp_guard(argc, argv);

  std::string log_dir;
  std::string out_dir;
  std::string params_yaml;
  std::optional<uint64_t> frame_filter;
  bool copy_reference = false;
  FirstOrderDubinsMppiCostParams cost_params;
  FirstOrderDubinsMppiVehicleParams vehicle_params;
  // Defaults closer to j6_gen2 for Autoware replay.
  vehicle_params.wheel_base = 4.76F;
  vehicle_params.ego_length = 5.0F;
  vehicle_params.ego_width = 1.9F;
  vehicle_params.ego_axle_to_box_center = 1.5F;
  vehicle_params.max_steer_angle = 0.7F;
  vehicle_params.steer_time_constant = 0.27F;
  vehicle_params.acc_time_constant = 0.1F;
  vehicle_params.vel_rate_lim = 7.0F;
  vehicle_params.steer_rate_lim = 5.0F;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char * name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + name);
      }
      return argv[++i];
    };
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    }
    if (arg == "--log-dir") {
      log_dir = need("--log-dir");
    } else if (arg == "--out-dir") {
      out_dir = need("--out-dir");
    } else if (arg == "--frame") {
      frame_filter = std::stoull(need("--frame"));
    } else if (arg == "--params-yaml") {
      params_yaml = need("--params-yaml");
    } else if (arg == "--set") {
      std::string key;
      std::string value;
      if (!parseKeyValue(need("--set"), key, value)) {
        throw std::runtime_error("Expected --set key=value");
      }
      applyCostParam(cost_params, key, std::stof(value));
    } else if (arg == "--wheel-base") {
      vehicle_params.wheel_base = std::stof(need("--wheel-base"));
    } else if (arg == "--ego-length") {
      vehicle_params.ego_length = std::stof(need("--ego-length"));
    } else if (arg == "--ego-width") {
      vehicle_params.ego_width = std::stof(need("--ego-width"));
    } else if (arg == "--max-steer-angle") {
      vehicle_params.max_steer_angle = std::stof(need("--max-steer-angle"));
    } else if (arg == "--steer-tau") {
      vehicle_params.steer_time_constant = std::stof(need("--steer-tau"));
    } else if (arg == "--copy-reference") {
      copy_reference = true;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      printUsage(argv[0]);
      return 1;
    }
  }

  if (log_dir.empty() || out_dir.empty()) {
    printUsage(argv[0]);
    return 1;
  }

  // Prefer params captured at log time; then yaml; then CLI --set / vehicle flags.
  loadCostParamsFromLog(log_dir, cost_params);
  loadVehicleParamsFromLog(log_dir, vehicle_params);

  if (!params_yaml.empty()) {
    loadParamsYaml(params_yaml, cost_params);
  }
  // Re-apply --set / vehicle CLI so they win over log + yaml.
  for (int i = 1; i + 1 < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--set") {
      std::string key;
      std::string value;
      if (parseKeyValue(argv[i + 1], key, value)) {
        applyCostParam(cost_params, key, std::stof(value));
      }
    } else if (arg == "--wheel-base") {
      vehicle_params.wheel_base = std::stof(argv[i + 1]);
    } else if (arg == "--ego-length") {
      vehicle_params.ego_length = std::stof(argv[i + 1]);
    } else if (arg == "--ego-width") {
      vehicle_params.ego_width = std::stof(argv[i + 1]);
    } else if (arg == "--max-steer-angle") {
      vehicle_params.max_steer_angle = std::stof(argv[i + 1]);
    } else if (arg == "--steer-tau") {
      vehicle_params.steer_time_constant = std::stof(argv[i + 1]);
    }
  }

  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (ec) {
    std::cerr << "Failed to create out-dir: " << ec.message() << "\n";
    return 1;
  }

  const auto frame_ids = listMppiDebugFrameIds(log_dir);
  if (frame_ids.empty()) {
    std::cerr << "No frames found in " << log_dir << "\n";
    return 1;
  }

  std::ofstream index_out(out_dir + "/index.csv");
  index_out << "frame_id,stamp_sec,stamp_nsec,baseline_cost,n_reference,n_optimized\n";

  size_t processed = 0;
  for (const uint64_t frame_id : frame_ids) {
    if (frame_filter && *frame_filter != frame_id) {
      continue;
    }
    const std::string tag = formatMppiDebugFrameId(frame_id);
    const std::string ref_path = log_dir + "/" + tag + "_reference.csv";
    Trajectory reference;
    if (!loadMppiDebugTrajectoryCsv(ref_path, reference)) {
      std::cerr << "Failed to load " << ref_path << "\n";
      return 1;
    }
    reference.header.frame_id = "map";

    const std::string ego_path = log_dir + "/" + tag + "_ego.csv";
    MppiDebugEgoState ego;
    Odometry odom;
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    autoware_vehicle_msgs::msg::SteeringReport steering;
    if (loadMppiDebugEgoCsv(ego_path, ego)) {
      odom = odometryFromEgo(ego, reference);
      accel = accelFromEgo(ego, reference);
      steering = steeringFromEgo(ego, reference);
    } else {
      static bool warned_missing_ego = false;
      if (!warned_missing_ego) {
        std::cerr
          << "WARNING: missing " << tag
          << "_ego.csv (and possibly others). Falling back to reference[0] as ego IC.\n"
             "Re-log with a build that writes *_ego.csv / cost_params.csv / vehicle_params.csv "
             "for faithful replay. Obstacles and road borders are still not logged.\n";
        warned_missing_ego = true;
      }
      odom = odometryFromReference(reference);
      accel = accelFromReference(reference);
      steering = steeringFromReference(reference);
    }
    const autoware_perception_msgs::msg::TrackedObjects empty_objects;

    // Fresh controller each frame so warm-start from other frames does not leak.
    FirstOrderDubinsMppiInterface frame_mppi;
    frame_mppi.setCostParams(cost_params);
    frame_mppi.setVehicleParams(vehicle_params);
    frame_mppi.setDebugTrajectoryLogging(false);

    const auto result =
      frame_mppi.optimizeTrajectory(reference, odom, accel, steering, empty_objects);

    const std::string opt_path = out_dir + "/" + tag + "_optimized.csv";
    if (!writeMppiDebugTrajectoryCsv(opt_path, result.debug.optimized_trajectory)) {
      std::cerr << "Failed to write " << opt_path << "\n";
      return 1;
    }

    // Stride keeps the CSV / histogram manageable (~4k samples from 32k rollouts).
    std::vector<float> raw_costs;
    std::vector<float> normalized_weights;
    constexpr int kCostVizStride = 8;
    if (frame_mppi.copySampleCostDistribution(raw_costs, normalized_weights, kCostVizStride)) {
      const std::string costs_path = out_dir + "/" + tag + "_costs.csv";
      if (!writeMppiDebugCostsCsv(costs_path, raw_costs, normalized_weights)) {
        std::cerr << "Failed to write " << costs_path << "\n";
        return 1;
      }
    } else {
      std::cerr << "WARNING: could not copy sample cost distribution for frame " << frame_id
                << "\n";
    }

    if (copy_reference) {
      const std::string out_ref = out_dir + "/" + tag + "_reference.csv";
      if (!writeMppiDebugTrajectoryCsv(out_ref, reference)) {
        std::cerr << "Failed to write " << out_ref << "\n";
        return 1;
      }
    }

    index_out << frame_id << "," << reference.header.stamp.sec << ","
              << reference.header.stamp.nanosec << "," << result.debug.baseline_cost << ","
              << reference.points.size() << "," << result.debug.optimized_trajectory.points.size()
              << "\n";
    ++processed;
    std::cout << "frame " << frame_id << " baseline_cost=" << result.debug.baseline_cost
              << " points=" << result.debug.optimized_trajectory.points.size() << "\n";
  }

  if (processed == 0U) {
    std::cerr << "No frames processed (check --frame filter).\n";
    return 1;
  }
  std::cout << "Wrote " << processed << " retuned frame(s) to " << out_dir << "\n";
  return 0;
}
