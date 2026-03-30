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

#include "autoware/boundary_departure_checker/footprints_generator.hpp"

#include <vector>

namespace autoware::boundary_departure_checker::footprints
{
using autoware_utils_geometry::pose2transform;
using autoware_utils_geometry::transform_vector;

FootprintMargin calc_margin_from_covariance(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale)
{
  const auto cov_in_map = covariance.covariance;
  Eigen::Matrix2d cov_xy_map;
  cov_xy_map << cov_in_map[0 * 6 + 0], cov_in_map[0 * 6 + 1], cov_in_map[1 * 6 + 0],
    cov_in_map[1 * 6 + 1];

  const double yaw_vehicle = tf2::getYaw(covariance.pose.orientation);

  // To get a position in a transformed coordinate, rotate the inverse direction
  Eigen::Matrix2d r_map2vehicle;
  r_map2vehicle << std::cos(-yaw_vehicle), -std::sin(-yaw_vehicle), std::sin(-yaw_vehicle),
    std::cos(-yaw_vehicle);
  // Rotate covariance E((X, Y)^t*(X, Y)) = E(R*(x,y)*(x,y)^t*R^t)
  // when Rotate point (X, Y)^t= R*(x, y)^t.
  const Eigen::Matrix2d cov_xy_vehicle = r_map2vehicle * cov_xy_map * r_map2vehicle.transpose();

  // The longitudinal/lateral length is represented
  // in cov_xy_vehicle(0,0), cov_xy_vehicle(1,1) respectively.
  return FootprintMargin{cov_xy_vehicle(0, 0) * scale, cov_xy_vehicle(1, 1) * scale};
}

Footprints generate(
  const std::vector<TrajectoryPoint> & trajectory_points,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::PoseWithCovariance & covariance)
{
  const auto margin = calc_margin_from_covariance(covariance, 0.0);
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat_m, margin.lon_m);

  Footprints footprints{};
  footprints.reserve(trajectory_points.size());
  for (const auto & pt : trajectory_points) {
    footprints.push_back(transform_vector(local_vehicle_footprint, pose2transform(pt.pose)));
  }
  return footprints;
}

std::vector<Side<autoware_utils_geometry::Segment2d>> get_sides_from_footprints(
  const Footprints & footprints)
{
  std::vector<Side<autoware_utils_geometry::Segment2d>> footprints_sides;
  footprints_sides.reserve(footprints.size());
  for (const auto & footprint : footprints) {
    const auto & right_front = footprint[vehicle_info_utils::VehicleInfo::FrontRightIndex];
    const auto & right_back = footprint[vehicle_info_utils::VehicleInfo::RearRightIndex];

    const auto & left_front = footprint[vehicle_info_utils::VehicleInfo::FrontLeftIndex];
    const auto & left_back = footprint[vehicle_info_utils::VehicleInfo::RearLeftIndex];

    Side<autoware_utils_geometry::Segment2d> side;
    side.right = {
      autoware_utils_geometry::Point2d(right_front.x(), right_front.y()),
      autoware_utils_geometry::Point2d(right_back.x(), right_back.y())};
    side.left = {
      autoware_utils_geometry::Point2d(left_front.x(), left_front.y()),
      autoware_utils_geometry::Point2d(left_back.x(), left_back.y())};

    footprints_sides.push_back(side);
  }
  return footprints_sides;
}

}  // namespace autoware::boundary_departure_checker::footprints
