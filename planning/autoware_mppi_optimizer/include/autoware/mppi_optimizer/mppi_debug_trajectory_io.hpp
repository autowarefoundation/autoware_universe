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

#ifndef AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_TRAJECTORY_IO_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_TRAJECTORY_IO_HPP_

#include "autoware/mppi_optimizer/mppi_debug_trajectory_logger.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::mppi_optimizer
{

inline std::string formatMppiDebugFrameId(const uint64_t id)
{
  std::ostringstream oss;
  oss << std::setw(6) << std::setfill('0') << id;
  return oss.str();
}

inline geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

inline bool writeMppiDebugTrajectoryCsv(
  const std::string & path, const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  std::ofstream out(path);
  if (!out) {
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

inline bool writeMppiDebugCostsCsv(
  const std::string & path, const std::vector<float> & raw_costs,
  const std::vector<float> & normalized_weights)
{
  if (raw_costs.size() != normalized_weights.size()) {
    return false;
  }
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "rollout_index,raw_cost,normalized_weight\n";
  out << std::setprecision(9) << std::fixed;
  for (size_t i = 0; i < raw_costs.size(); ++i) {
    out << i << "," << raw_costs[i] << "," << normalized_weights[i] << "\n";
  }
  return true;
}

inline bool loadMppiDebugTrajectoryCsv(
  const std::string & path, autoware_planning_msgs::msg::Trajectory & trajectory)
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  trajectory.points.clear();
  std::string line;
  if (!std::getline(in, line)) {
    return false;
  }
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> vals;
    while (std::getline(ss, cell, ',')) {
      vals.push_back(std::stod(cell));
    }
    if (vals.size() < 9U) {
      continue;
    }
    autoware_planning_msgs::msg::TrajectoryPoint point;
    const double t = vals[0];
    point.time_from_start.sec = static_cast<int32_t>(t);
    point.time_from_start.nanosec = static_cast<uint32_t>((t - point.time_from_start.sec) * 1.0e9);
    point.pose.position.x = vals[1];
    point.pose.position.y = vals[2];
    point.pose.position.z = vals[3];
    point.pose.orientation = quaternionFromYaw(vals[4]);
    point.longitudinal_velocity_mps = static_cast<float>(vals[5]);
    point.acceleration_mps2 = static_cast<float>(vals[6]);
    point.front_wheel_angle_rad = static_cast<float>(vals[7]);
    point.heading_rate_rps = static_cast<float>(vals[8]);
    trajectory.points.push_back(point);
  }
  return !trajectory.points.empty();
}

inline bool loadMppiDebugEgoCsv(const std::string & path, MppiDebugEgoState & ego)
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::string line;
  if (!std::getline(in, line)) {
    return false;
  }
  if (!std::getline(in, line) || line.empty()) {
    return false;
  }
  std::stringstream ss(line);
  std::string cell;
  std::vector<double> vals;
  while (std::getline(ss, cell, ',')) {
    vals.push_back(std::stod(cell));
  }
  if (vals.size() < 7U) {
    return false;
  }
  ego.x = vals[0];
  ego.y = vals[1];
  ego.z = vals[2];
  ego.yaw = vals[3];
  ego.v = vals[4];
  ego.accel = vals[5];
  ego.steer = vals[6];
  return true;
}

inline bool loadMppiDebugKeyValueCsv(
  const std::string & path, std::unordered_map<std::string, float> & out)
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::string line;
  std::getline(in, line);  // header
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    const auto comma = line.find(',');
    if (comma == std::string::npos) {
      continue;
    }
    out[line.substr(0, comma)] = std::stof(line.substr(comma + 1));
  }
  return !out.empty();
}

inline std::vector<uint64_t> listMppiDebugFrameIds(const std::string & log_dir)
{
  std::vector<uint64_t> ids;
  const std::filesystem::path index_path = std::filesystem::path(log_dir) / "index.csv";
  if (std::filesystem::exists(index_path)) {
    std::ifstream in(index_path);
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
      if (line.empty()) {
        continue;
      }
      const auto comma = line.find(',');
      ids.push_back(std::stoull(comma == std::string::npos ? line : line.substr(0, comma)));
    }
    return ids;
  }

  for (const auto & entry : std::filesystem::directory_iterator(log_dir)) {
    const auto name = entry.path().filename().string();
    constexpr char kSuffix[] = "_reference.csv";
    if (name.size() > sizeof(kSuffix) - 1U &&
        name.compare(name.size() - (sizeof(kSuffix) - 1U), sizeof(kSuffix) - 1U, kSuffix) == 0) {
      ids.push_back(std::stoull(name.substr(0, name.size() - (sizeof(kSuffix) - 1U))));
    }
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_TRAJECTORY_IO_HPP_
