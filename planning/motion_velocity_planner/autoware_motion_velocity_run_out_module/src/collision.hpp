// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef COLLISION_HPP_
#define COLLISION_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief time intervals for ego and the object when their predicted paths overlap
struct TimeOverlapIntervalPair
{
  TimeOverlapInterval ego;
  TimeOverlapInterval object;

  TimeOverlapIntervalPair(TimeOverlapInterval e, TimeOverlapInterval o)
  : ego(std::move(e)), object(std::move(o))
  {
  }
};

/// @brief calculate the footprint intersection information between an object segment and an ego
/// vehicle footprint segment This function determines the intersection point between a segment of
/// the predicted object's path and a segment of the ego vehicle's trajectory footprint.  It
/// calculates the times of the intersection for both the ego vehicle and the predicted object, as
/// well as the position of the intersection relative to the ego vehicle's footprint.
///
/// @param[in] object_segment The segment of the predicted object's path (a line segment).
/// @param[in] intersection_point The point where the object segment intersects the ego vehicle's
/// footprint segment.
/// @param[in] ego_query_result The result of a spatial query on the ego vehicle's footprint,
/// containing the intersecting ego vehicle footprint segment and related data.
/// @param[in] ego_trajectory The ego vehicle's trajectory, represented as a vector of trajectory
/// points.
/// @param[in] object_segment_times The start and end times of the object segment.
/// @return A FootprintIntersection struct containing information about the intersection.
FootprintIntersection calculate_footprint_intersection(
  const universe_utils::Segment2d & object_segment,
  const universe_utils::Point2d & intersection_point, const FootprintSegmentNode & ego_query_result,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const std::pair<double, double> object_segment_times);

/// @brief calculate the interpolated trajectory point closest to the given position
/// @details the pose (position + orientation), time from start, and velocities are interpolated
/// @param [in] trajectory trajectory with time information
/// @param [in] p target position
/// @param [in] longitudinal_offset [m] arc length offset for which to calculate the interpolation
/// (default to 0)
std::pair<autoware_planning_msgs::msg::TrajectoryPoint, double>
calculate_closest_interpolated_point_and_arc_length(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const universe_utils::Point2d & p, const double longitudinal_offset = 0.0);

/// @brief Calculate the intersection between the end point of a linestring and the ego vehicle's
/// trajectory footprint
/// @param[in] ls The corner linestring of the predicted object.
/// @param[in] footprint The footprint of the ego vehicle's trajectory.
/// @param[in] ego_trajectory The ego vehicle's trajectory.
/// @param[in] ls_time_step The time step of the predicted object's linestring.
/// @param[in] check_front If true, the first point of the linestring is checked, otherwise the last
/// point is checked
std::optional<FootprintIntersection> calculate_end_point_intersection(
  const universe_utils::LineString2d & ls, const TrajectoryCornerFootprint & footprint,
  const double ls_time_step, const bool check_front);

/// @brief Calculates intersections between a predicted object's corner linestring and the ego
/// vehicle's trajectory footprint.
/// @param[in] ls The corner linestring of the predicted object.
/// @param[in] footprint The footprint of the ego vehicle's trajectory.
/// @param[in] ego_trajectory The ego vehicle's trajectory.
/// @param[in] ls_time_step The time step of the predicted object's linestring.
/// @return A vector of footprint intersections.
std::vector<FootprintIntersection> calculate_intersections(
  const universe_utils::LineString2d & ls, const TrajectoryCornerFootprint & footprint,
  const double ls_time_step);

/// @brief group the intersections into overlap intervals of increasing object time
/// @param intersections intersection points calculated between one object footprint linestring and
/// the ego footprint
/// @return the corresponding overlap time intervals for both ego and the object
std::vector<TimeOverlapIntervalPair> calculate_overlap_intervals(
  std::vector<FootprintIntersection> intersections);

/// @brief calculate a collision from the time when the trajectories of ego and an object overlap
/// @return a Collision object with the collision type and predicted collision time
Collision calculate_collision(
  const TimeOverlapInterval & ego, const TimeOverlapInterval & object, const Parameters & params);

/// @brief calculate a collision when the ego and object are predicted to overlap at the same time
void calculate_overlapping_collision(
  Collision & c, const TimeOverlapInterval & ego, const TimeOverlapInterval & object,
  const Parameters & params);

/// @brief Calculates collisions based on overlap intervals.
///
/// @param[in] intervals A vector of time collision interval pairs.
/// @param[in] params The parameters for collision calculation.
/// @return A vector of collisions.
std::vector<Collision> calculate_interval_collisions(
  std::vector<TimeOverlapIntervalPair> intervals, const Parameters & params);

/// @brief Calculates collisions between the ego vehicle's trajectory footprint and a predicted
/// object's footprint.
///
/// This function iterates through the corner footprints of the predicted object, calculates
/// intersections with the ego vehicle's footprint, determines overlap intervals, and calculates
/// collisions based on those intervals.
///
/// @param[in] ego_footprint The footprint of the ego vehicle's trajectory.
/// @param[in] object_footprint The footprint of the predicted object.
/// @param[in] ego_trajectory The ego vehicle's trajectory.
/// @param[in] filtering_data Data containing the polygons where collisions should be ignored
/// @param[in] min_arc_length [m] minimum arc length along the ego trajectory where collisions are
/// considered
/// @param[in] params The parameters for collision calculation.
/// @return A vector of collisions.
std::vector<Collision> calculate_footprint_collisions(
  const TrajectoryCornerFootprint & ego_footprint,
  const ObjectPredictedPathFootprint & object_footprint, const FilteringData & filtering_data,
  const double min_arc_length, const Parameters & params);

/// @brief Calculates collisions between the ego vehicle's trajectory and a predicted object.
/// @param[inout] object A predicted object.  Collisions are added to each object.
/// @param[in] ego_footprint The footprint of the ego vehicle's trajectory.
/// @param[in] ego_trajectory The ego vehicle's trajectory.
/// @param[in] filtering_data Data containing the polygons where collisions should be ignored
/// @param[in] min_arc_length [m] minimum arc length where collisions are considered
/// @param[in] params The parameters for collision calculation.
void calculate_object_collisions(
  std::vector<Object> & objects, const TrajectoryCornerFootprint & ego_footprint,
  const FilteringDataPerLabel & filtering_data, const double min_arc_length,
  const Parameters & params);

/// @brief Calculates collisions between the ego vehicle's trajectory and a set of predicted
/// objects.
/// @param[inout] objects A vector of predicted objects.  Collisions are added to each object.
/// @param[in] ego_footprint The footprint of the ego vehicle's trajectory.
/// @param[in] ego_trajectory The ego vehicle's trajectory.
/// @param[in] filtering_data Data containing the polygons where collisions should be ignored
/// @param[in] min_arc_length [m] minimum arc length where collisions are considered
/// @param[in] params The parameters for collision calculation.
void calculate_collisions(
  std::vector<Object> & objects, const TrajectoryCornerFootprint & ego_footprint,
  const FilteringDataPerLabel & filtering_data, const double min_arc_length,
  const Parameters & params);
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // COLLISION_HPP_
