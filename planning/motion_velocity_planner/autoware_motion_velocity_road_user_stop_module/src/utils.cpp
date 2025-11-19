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
#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <tf2/utils.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>

#include <limits>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::road_user_stop::utils
{

using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

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
  using autoware::motion_velocity_planner::polygon_utils::calculate_error_poses;
  using autoware_utils_geometry::calc_offset_pose;
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
      const double half_width = vehicle_info.vehicle_width_m / 2.0;
      const double front_length = vehicle_info.max_longitudinal_offset_m;
      const auto left_point =
        calc_offset_pose(pose, front_length, half_width + lat_margin, 0.0).position;
      const auto right_point =
        calc_offset_pose(pose, front_length, -half_width - lat_margin, 0.0).position;
      current_bumper_points.push_back(Point2d{left_point.x, left_point.y});
      current_bumper_points.push_back(Point2d{right_point.x, right_point.y});
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
