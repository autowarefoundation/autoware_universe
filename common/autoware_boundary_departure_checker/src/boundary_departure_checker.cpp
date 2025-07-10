// Copyright 2020 Tier IV, Inc.
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

#include "autoware/boundary_departure_checker/boundary_departure_checker.hpp"

#include "autoware/boundary_departure_checker/conversion.hpp"
#include "autoware/boundary_departure_checker/utils.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/view.hpp>
#include <tl_expected/expected.hpp>

#include <boost/geometry.hpp>

#include <fmt/format.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_utils::Point2d;
using geometry_msgs::msg::Point;

double calcBrakingDistance(
  const double abs_velocity, const double max_deceleration, const double delay_time)
{
  return (abs_velocity * abs_velocity) / (2.0 * max_deceleration) + delay_time * abs_velocity;
}

bool isInAnyLane(const lanelet::ConstLanelets & candidate_lanelets, const Point2d & point)
{
  return std::any_of(
    candidate_lanelets.begin(), candidate_lanelets.end(), [&](const lanelet::ConstLanelet & ll) {
      return boost::geometry::within(point, ll.polygon2d().basicPolygon());
    });
}

}  // namespace

namespace autoware::boundary_departure_checker
{
Output BoundaryDepartureChecker::update(const Input & input)
{
  Output output{};

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  {
    constexpr double min_velocity = 0.01;
    const auto & raw_abs_velocity = std::abs(input.current_odom->twist.twist.linear.x);
    const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;

    const auto braking_distance = std::max(
      param_.min_braking_distance,
      calcBrakingDistance(abs_velocity, param_.max_deceleration, param_.delay_time));

    output.resampled_trajectory = utils::cutTrajectory(
      utils::resampleTrajectory(*input.predicted_trajectory, param_.resample_interval),
      braking_distance);
    output.processing_time_map["resampleTrajectory"] = stop_watch.toc(true);
  }
  output.vehicle_footprints = utils::createVehicleFootprints(
    input.current_odom->pose, output.resampled_trajectory, *vehicle_info_ptr_,
    param_.footprint_margin_scale);
  output.processing_time_map["createVehicleFootprints"] = stop_watch.toc(true);

  output.vehicle_passing_areas = utils::createVehiclePassingAreas(output.vehicle_footprints);
  output.processing_time_map["createVehiclePassingAreas"] = stop_watch.toc(true);

  const auto candidate_road_lanelets =
    utils::getCandidateLanelets(input.route_lanelets, output.vehicle_footprints);
  const auto candidate_shoulder_lanelets =
    utils::getCandidateLanelets(input.shoulder_lanelets, output.vehicle_footprints);
  output.candidate_lanelets = candidate_road_lanelets;
  output.candidate_lanelets.insert(
    output.candidate_lanelets.end(), candidate_shoulder_lanelets.begin(),
    candidate_shoulder_lanelets.end());

  output.processing_time_map["getCandidateLanelets"] = stop_watch.toc(true);

  output.will_leave_lane = willLeaveLane(output.candidate_lanelets, output.vehicle_footprints);
  output.processing_time_map["willLeaveLane"] = stop_watch.toc(true);

  output.is_out_of_lane = isOutOfLane(output.candidate_lanelets, output.vehicle_footprints.front());
  output.processing_time_map["isOutOfLane"] = stop_watch.toc(true);

  const double max_search_length_for_boundaries =
    utils::calcMaxSearchLengthForBoundaries(*input.predicted_trajectory, *vehicle_info_ptr_);
  const auto uncrossable_boundaries = extractUncrossableBoundaries(
    *input.lanelet_map, input.predicted_trajectory->points.front().pose.position,
    max_search_length_for_boundaries, input.boundary_types_to_detect);
  output.will_cross_boundary = willCrossBoundary(output.vehicle_footprints, uncrossable_boundaries);
  output.processing_time_map["willCrossBoundary"] = stop_watch.toc(true);

  return output;
}

bool BoundaryDepartureChecker::checkPathWillLeaveLane(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  lanelet::ConstLanelets candidate_lanelets =
    utils::getCandidateLanelets(lanelets, vehicle_footprints);
  return willLeaveLane(candidate_lanelets, vehicle_footprints);
}

bool BoundaryDepartureChecker::willLeaveLane(
  const lanelet::ConstLanelets & candidate_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & vehicle_footprint : vehicle_footprints) {
    if (isOutOfLane(candidate_lanelets, vehicle_footprint)) {
      return true;
    }
  }

  return false;
}

std::vector<std::pair<double, lanelet::Lanelet>> BoundaryDepartureChecker::getLaneletsFromPath(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // Get Footprint Hull basic polygon
  std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  LinearRing2d footprint_hull = utils::createHullFromFootprints(vehicle_footprints);

  auto footprint_hull_basic_polygon = utils::to_basic_polygon_2d(footprint_hull);

  // Find all lanelets that intersect the footprint hull
  return lanelet::geometry::findWithin2d(
    lanelet_map_ptr->laneletLayer, footprint_hull_basic_polygon, 0.0);
}

std::optional<autoware_utils::Polygon2d> BoundaryDepartureChecker::getFusedLaneletPolygonForPath(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto lanelets_distance_pair = getLaneletsFromPath(lanelet_map_ptr, path);
  if (lanelets_distance_pair.empty()) return std::nullopt;

  // Fuse lanelets into a single BasicPolygon2d
  autoware_utils::MultiPolygon2d lanelet_unions;
  autoware_utils::MultiPolygon2d result;

  for (size_t i = 0; i < lanelets_distance_pair.size(); ++i) {
    const auto & route_lanelet = lanelets_distance_pair.at(i).second;
    const auto & p = route_lanelet.polygon2d().basicPolygon();
    auto poly = utils::to_polygon_2d(p);
    boost::geometry::union_(lanelet_unions, poly, result);
    lanelet_unions = result;
    result.clear();
  }

  if (lanelet_unions.empty()) return std::nullopt;
  return lanelet_unions.front();
}

bool BoundaryDepartureChecker::updateFusedLaneletPolygonForPath(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
  std::vector<lanelet::Id> & fused_lanelets_id,
  std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto lanelets_distance_pair = getLaneletsFromPath(lanelet_map_ptr, path);
  if (lanelets_distance_pair.empty()) return false;

  autoware_utils::MultiPolygon2d lanelet_unions;
  autoware_utils::MultiPolygon2d result;

  if (fused_lanelets_polygon) lanelet_unions.push_back(fused_lanelets_polygon.value());

  for (size_t i = 0; i < lanelets_distance_pair.size(); ++i) {
    const auto & route_lanelet = lanelets_distance_pair.at(i).second;
    bool id_exist = std::any_of(
      fused_lanelets_id.begin(), fused_lanelets_id.end(),
      [&](const auto & id) { return id == route_lanelet.id(); });

    if (id_exist) continue;

    const auto & p = route_lanelet.polygon2d().basicPolygon();
    auto poly = utils::to_polygon_2d(p);
    boost::geometry::union_(lanelet_unions, poly, result);
    lanelet_unions = result;
    result.clear();
    fused_lanelets_id.push_back(route_lanelet.id());
  }

  if (lanelet_unions.empty()) {
    fused_lanelets_polygon = std::nullopt;
    return false;
  }

  fused_lanelets_polygon = lanelet_unions.front();
  return true;
}

bool BoundaryDepartureChecker::checkPathWillLeaveLane(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // check if the footprint is not fully contained within the fused lanelets polygon
  const std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  const auto fused_lanelets_polygon = getFusedLaneletPolygonForPath(lanelet_map_ptr, path);
  if (!fused_lanelets_polygon) return true;
  return !std::all_of(
    vehicle_footprints.begin(), vehicle_footprints.end(),
    [&fused_lanelets_polygon](const auto & footprint) {
      return boost::geometry::within(footprint, fused_lanelets_polygon.value());
    });
}

bool BoundaryDepartureChecker::checkPathWillLeaveLane(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
  std::vector<lanelet::Id> & fused_lanelets_id,
  std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);

  auto is_all_footprints_within = [&](const auto & polygon) {
    return std::all_of(
      vehicle_footprints.begin(), vehicle_footprints.end(),
      [&polygon](const auto & footprint) { return boost::geometry::within(footprint, polygon); });
  };

  // If lanelets polygon exists and all footprints are within it, the path doesn't leave the lane
  if (fused_lanelets_polygon && is_all_footprints_within(fused_lanelets_polygon.value())) {
    return false;
  }

  // Update the lanelet polygon for the current path
  if (!updateFusedLaneletPolygonForPath(
        lanelet_map_ptr, path, fused_lanelets_id, fused_lanelets_polygon)) {
    // If update fails, assume the path leaves the lane
    return true;
  }

  // Check if any footprint is outside the updated lanelets polygon
  return !is_all_footprints_within(fused_lanelets_polygon.value());
}

PathWithLaneId BoundaryDepartureChecker::cropPointsOutsideOfLanes(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path, const size_t end_index,
  std::vector<lanelet::Id> & fused_lanelets_id,
  std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  PathWithLaneId temp_path;

  // Update the lanelet polygon for the current path
  if (
    path.points.empty() || !updateFusedLaneletPolygonForPath(
                             lanelet_map_ptr, path, fused_lanelets_id, fused_lanelets_polygon)) {
    return temp_path;
  }

  const auto vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);

  {
    autoware_utils::ScopedTimeTrack st2(
      "check if footprint is within fused_lanelets_polygon", *time_keeper_);

    size_t idx = 0;
    std::for_each(
      vehicle_footprints.begin(), vehicle_footprints.end(), [&](const auto & footprint) {
        if (idx > end_index || boost::geometry::within(footprint, fused_lanelets_polygon.value())) {
          temp_path.points.push_back(path.points.at(idx));
        }
        ++idx;
      });
  }
  PathWithLaneId cropped_path = path;
  cropped_path.points = temp_path.points;
  return cropped_path;
}

bool BoundaryDepartureChecker::isOutOfLane(
  const lanelet::ConstLanelets & candidate_lanelets, const LinearRing2d & vehicle_footprint)
{
  for (const auto & point : vehicle_footprint) {
    if (!isInAnyLane(candidate_lanelets, point)) {
      return true;
    }
  }

  return false;
}

bool BoundaryDepartureChecker::willCrossBoundary(
  const std::vector<LinearRing2d> & vehicle_footprints,
  const SegmentRtree & uncrossable_segments) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & footprint : vehicle_footprints) {
    std::vector<Segment2d> intersection_result;
    uncrossable_segments.query(
      boost::geometry::index::intersects(footprint), std::back_inserter(intersection_result));
    if (!intersection_result.empty()) {
      return true;
    }
  }
  return false;
}
}  // namespace autoware::boundary_departure_checker
