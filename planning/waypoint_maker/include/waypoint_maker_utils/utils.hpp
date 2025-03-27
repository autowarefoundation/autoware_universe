#ifndef WAYPOINT_MAKER_UTILS__UTILS_HPP_
#define WAYPOINT_MAKER_UTILS__UTILS_HPP_

#include "conversion.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <filesystem>  // c++ 17
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace waypoint_maker
{

size_t countColumns(const std::string & line)
{
  std::istringstream ss(line);
  size_t ncol = 0;

  std::string column;
  while (std::getline(ss, column, ',')) ++ncol;

  return ncol;
}

void parseColumns(const std::string & line, std::vector<std::string> * columns)
{
  std::istringstream ss(line);
  std::string column;
  while (std::getline(ss, column, ',')) {
    while (1) {
      auto res = std::find(column.begin(), column.end(), ' ');
      if (res == column.end()) break;

      column.erase(res);
    }
    if (!column.empty()) columns->emplace_back(column);
  }
}

bool verifyFileConsistency(const char * filename)
{
  std::ifstream ifs(filename);

  if (!ifs) return false;

  std::string line;
  std::getline(ifs, line);  // remove first line

  constexpr size_t ncol = 5;  // x, y, z, yaw, velocity

  while (std::getline(ifs, line)) {
    if (waypoint_maker::countColumns(line) != ncol) {
      return false;
    }
  }  // search from second line

  return true;
}

void parseWaypoint(
  const std::string & line, autoware_planning_msgs::msg::TrajectoryPoint * point,
  const bool & max_velocity_bool, const double & max_velocity)
{
  std::vector<std::string> columns;
  parseColumns(line, &columns);

  point->pose.position.x = std::stod(columns[0]);
  point->pose.position.y = std::stod(columns[1]);
  point->pose.position.z = std::stod(columns[2]);
  point->pose.orientation = getQuaternionFromYaw(std::stod(columns[3]));
  if (max_velocity_bool) {
    point->longitudinal_velocity_mps = max_velocity;
  } else {
    point->longitudinal_velocity_mps = kmph2mps(std::stod(columns[4]));
  }
}

void loadWaypoints(
  const char * filename, std::vector<autoware_planning_msgs::msg::TrajectoryPoint> * points,
  const bool & max_velocity_bool, const double & max_velocity)
{
  std::ifstream ifs(filename);

  if (!ifs) {
    // RCLCPP_ERROR(this->get_logger(), "Lane data is something wrong.");
    return;
  }

  std::string line;
  std::getline(ifs, line);  // Remove first line

  while (std::getline(ifs, line)) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    parseWaypoint(line, &point, max_velocity_bool, max_velocity);
    points->emplace_back(point);
  }
}

autoware_planning_msgs::msg::Trajectory createWaypointTrajectory(
  const std::string & file_path, const bool & max_velocity_bool, const double & max_velocity)
{
  autoware_planning_msgs::msg::Trajectory output;
  output.header.frame_id = "map";
  output.header.stamp = rclcpp::Time();

  if (!verifyFileConsistency(file_path.c_str())) {
    return output;
  }

  RCLCPP_INFO(rclcpp::get_logger("waypoint_maker"), "Waypoint data is valid. Publishing");

  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  loadWaypoints(file_path.c_str(), &points, max_velocity_bool, max_velocity);

  for (const auto & point : points) output.points.push_back(point);

  return output;
}

const std::string currentDateTime()
{
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tstruct);

  return buf;
}

// Check save log folder
bool dirExists(const std::string & path)
{
  struct stat info;
  if (stat(path.c_str(), &info) == 0 && info.st_mode & S_IFDIR) {
    return true;
  }
  return false;
}

}  // namespace waypoint_maker

#endif  // WAYPOINT_MAKER_UTILS__UTILS_HPP_
