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

#include "utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>

#include <tf2/utils.h>

#include <limits>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::road_user_stop::utils
{
namespace
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

inline Point2d msg_to_2d(const geometry_msgs::msg::Point & point)
{
  return Point2d{point.x, point.y};
}

// create front bumper line segment (not a polygon, just the front line)
std::pair<Point2d, Point2d> create_front_bumper_line(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double left_margin,
  const double right_margin)
{
  using autoware_utils_geometry::calc_offset_pose;
  const double half_width = vehicle_info.vehicle_width_m / 2.0;
  const double front_length = vehicle_info.max_longitudinal_offset_m;

  // create front bumper line (left to right)
  // point0: front left
  const auto point0 = calc_offset_pose(pose, front_length, half_width + left_margin, 0.0).position;
  // point1: front right
  const auto point1 =
    calc_offset_pose(pose, front_length, -half_width - right_margin, 0.0).position;

  return {msg_to_2d(point0), msg_to_2d(point1)};
}

// estimate the future ego pose with assuming that the pose error against the reference path will
// decrease to zero by the time_to_convergence.
std::vector<geometry_msgs::msg::Pose> calculate_error_poses(
  const std::vector<TrajectoryPoint> & traj_points,
  const geometry_msgs::msg::Pose & current_ego_pose, const double time_to_convergence)
{
  std::vector<geometry_msgs::msg::Pose> error_poses;
  error_poses.reserve(traj_points.size());

  const size_t nearest_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, current_ego_pose.position);
  const auto nearest_pose = traj_points.at(nearest_idx).pose;
  const auto current_ego_pose_error =
    autoware_utils_geometry::inverse_transform_pose(current_ego_pose, nearest_pose);
  const double current_ego_lat_error = current_ego_pose_error.position.y;
  const double current_ego_yaw_error = tf2::getYaw(current_ego_pose_error.orientation);
  double time_elapsed{0.0};

  for (size_t i = 0; i < traj_points.size(); ++i) {
    if (time_elapsed >= time_to_convergence) {
      break;
    }

    const double rem_ratio = (time_to_convergence - time_elapsed) / time_to_convergence;
    geometry_msgs::msg::Pose indexed_pose_err;
    indexed_pose_err.set__orientation(
      autoware_utils_geometry::create_quaternion_from_yaw(current_ego_yaw_error * rem_ratio));
    indexed_pose_err.set__position(
      autoware_utils_geometry::create_point(0.0, current_ego_lat_error * rem_ratio, 0.0));
    error_poses.push_back(
      autoware_utils_geometry::transform_pose(indexed_pose_err, traj_points.at(i).pose));

    if (traj_points.at(i).longitudinal_velocity_mps != 0.0 && i < traj_points.size() - 1) {
      time_elapsed += autoware_utils_geometry::calc_distance2d(
                        traj_points.at(i).pose.position, traj_points.at(i + 1).pose.position) /
                      std::abs(traj_points.at(i).longitudinal_velocity_mps);
    } else {
      time_elapsed = std::numeric_limits<double>::max();
    }
  }
  return error_poses;
}

}  // namespace

autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & poly)
{
  autoware_utils_geometry::Polygon2d polygon;
  auto & outer = polygon.outer();

  outer.reserve(poly.size());
  for (const auto & p : poly) {
    outer.emplace_back(p.x(), p.y());
  }
  boost::geometry::correct(polygon);
  return polygon;
}

autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon3d & poly)
{
  autoware_utils_geometry::Polygon2d polygon;
  auto & outer = polygon.outer();

  outer.reserve(poly.size());
  for (const auto & p : poly) {
    outer.emplace_back(p.x(), p.y());  // ignore z-coordinate
  }
  boost::geometry::correct(polygon);
  return polygon;
}

std::vector<Polygon2d> create_one_step_polygons_from_front(
  const std::vector<TrajectoryPoint> & traj_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  [[maybe_unused]] const double decimate_trajectory_step_length)
{
  const auto error_poses =
    enable_to_consider_current_pose
      ? calculate_error_poses(traj_points, current_ego_pose, time_to_convergence)
      : std::vector<geometry_msgs::msg::Pose>{};

  std::vector<Polygon2d> output_polygons;
  std::pair<Point2d, Point2d> prev_bumper_line;

  for (size_t i = 0; i < traj_points.size(); ++i) {
    // get current trajectory pose
    std::vector<geometry_msgs::msg::Pose> current_poses = {traj_points.at(i).pose};
    if (i < error_poses.size()) {
      current_poses.push_back(error_poses.at(i));
    }

    // collect all front bumper points for current trajectory point
    std::vector<Point2d> current_bumper_points;
    for (const auto & pose : current_poses) {
      const auto bumper_line = create_front_bumper_line(pose, vehicle_info, lat_margin, lat_margin);
      current_bumper_points.push_back(bumper_line.first);   // left point
      current_bumper_points.push_back(bumper_line.second);  // right point
    }

    if (i == 0) {
      // for the first point, create a small polygon just at the front bumper
      Polygon2d first_polygon;
      for (const auto & point : current_bumper_points) {
        boost::geometry::append(first_polygon, point);
      }
      // close the polygon
      if (!current_bumper_points.empty()) {
        boost::geometry::append(first_polygon, current_bumper_points.front());
      }
      boost::geometry::correct(first_polygon);
      output_polygons.push_back(first_polygon);

      // store for next iteration
      if (current_bumper_points.size() >= 2) {
        prev_bumper_line = {current_bumper_points[0], current_bumper_points[1]};
      }
    } else {
      // create polygon connecting previous and current front bumper lines
      // this creates a polygon that extends forward from the previous bumper
      Polygon2d segment_polygon;

      // add previous bumper line points
      boost::geometry::append(segment_polygon, prev_bumper_line.first);   // prev left
      boost::geometry::append(segment_polygon, prev_bumper_line.second);  // prev right

      // add current bumper line points (in reverse order to close properly)
      if (current_bumper_points.size() >= 2) {
        boost::geometry::append(segment_polygon, current_bumper_points[1]);  // current right
        boost::geometry::append(segment_polygon, current_bumper_points[0]);  // current left
      }

      // close polygon
      boost::geometry::append(segment_polygon, prev_bumper_line.first);

      boost::geometry::correct(segment_polygon);
      output_polygons.push_back(segment_polygon);

      // update previous bumper line for next iteration
      if (current_bumper_points.size() >= 2) {
        prev_bumper_line = {current_bumper_points[0], current_bumper_points[1]};
      }
    }
  }
  return output_polygons;
}

}  // namespace autoware::motion_velocity_planner::road_user_stop::utils
